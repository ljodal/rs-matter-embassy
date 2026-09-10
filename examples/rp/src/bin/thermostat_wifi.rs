//! An example utilizing the `EmbassyWifiMatterStack` struct to expose several
//! simulated heating zones as a Matter bridge.
//!
//! Like `light_wifi`, this uses Wifi as the main transport and BLE for
//! commissioning. The interesting part is the data model rather than the
//! transport:
//!
//! ```text
//! ep0        Root node       (the hidden Matter system clusters)
//! ep1        Aggregator      (Descriptor only)
//! ep2..      Bridged Node + Thermostat, one per zone
//!            (Descriptor, BridgedDeviceBasicInformation, Identify, Thermostat)
//! ```
//!
//! The bridge shape - rather than bare Thermostat endpoints - is what a
//! real "one MCU, several remote sensors" device wants: each zone gets its own
//! `NodeLabel` (so it shows up named in the ecosystem app rather than as
//! "Thermostat 2") and its own `Reachable` flag, which is how you tell a
//! controller that a sensor has gone silent.
//!
//! `rs-matter` has no hand-written Thermostat cluster, but its `build.rs`
//! generates every cluster in the Matter IDL into `dm::clusters::decl`, so
//! `decl::thermostat::ClusterHandler` is there to be implemented - which is
//! what [`ThermostatZone`] below does. There is no `ThermostatHooks`
//! equivalent to `OnOffHooks`, so the cluster's behaviour (setpoint clamping,
//! `SetpointRaiseLower`) is implemented here too.
//!
//! The zones are heat-only (the `HEATING` feature, no cooling) and simulated:
//! each zone's local temperature drifts slowly towards its heating setpoint
//! while its system mode is `Heat`, and towards an ambient temperature while
//! it is `Off`. So writing a setpoint from a controller produces visible
//! feedback, and subscriptions get a steady trickle of attribute reports.
#![no_std]
#![no_main]
#![recursion_limit = "256"]

use core::mem::MaybeUninit;
use core::pin::pin;
use core::ptr::addr_of_mut;

#[cfg(not(feature = "skip-cyw43-firmware"))]
use cyw43::{aligned_bytes, Aligned, A4};
use embassy_executor::Spawner;

use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::dma;
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIO0};
use embassy_rp::pio::InterruptHandler;
use embassy_rp::usb::Driver as UsbDriver;

use embassy_time::{Duration, Timer};

use embedded_alloc::LlffHeap;

use heapless::String;

// Pulls in the C malloc family the NimBLE BLE host allocates through
#[cfg(feature = "nimble")]
use tinyrlibc as _;

use log::info;

use rs_matter_embassy::matter::crypto::{default_crypto, Crypto};
use rs_matter_embassy::matter::dm::clusters::basic_info::BasicInfoConfig;
use rs_matter_embassy::matter::dm::clusters::decl::bridged_device_basic_information as bdbi;
use rs_matter_embassy::matter::dm::clusters::decl::thermostat;
use rs_matter_embassy::matter::dm::clusters::desc::{self, ClusterHandler as _};
use rs_matter_embassy::matter::dm::clusters::identify;
use rs_matter_embassy::matter::dm::devices::test::{
    DAC_PRIVKEY, TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET,
};
use rs_matter_embassy::matter::dm::devices::{DEV_TYPE_AGGREGATOR, DEV_TYPE_BRIDGED_NODE};
use rs_matter_embassy::matter::dm::{
    Async, Cluster, Dataver, DeviceType, EmptyHandler, Endpoint, EpClMatcher, HandlerContext,
    InvokeContext, Node, ReadContext, WriteContext,
};
use rs_matter_embassy::matter::error::{Error, ErrorCode};
use rs_matter_embassy::matter::persist::DummyKvBlobStore;
use rs_matter_embassy::matter::tlv::{Nullable, TLVBuilderParent, Utf8Str, Utf8StrBuilder};
use rs_matter_embassy::matter::utils::cell::RefCell;
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::select::Coalesce;
use rs_matter_embassy::matter::utils::sync::blocking::Mutex;
use rs_matter_embassy::matter::{clusters, devices, with};
use rs_matter_embassy::stack::network::Network;
use rs_matter_embassy::stack::rand::reseeding_csprng;
use rs_matter_embassy::wireless::rp::RpWifiDriver;
use rs_matter_embassy::wireless::{EmbassyWifi, EmbassyWifiMatterStack};

use rp_examples::{logger_task, pairing_reminder, report_last_panic, UsbIrqs};

macro_rules! mk_static {
    ($t:ty) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.uninit()
    }};
    ($t:ty,$val:expr) => {{
        mk_static!($t).write($val)
    }};
}

/// The metadata for one bridged zone endpoint.
///
/// A macro rather than a `const fn`, because `devices!` and `clusters!` expand
/// to `&[..]` literals, which are only promoted to `'static` when they appear
/// in a const context - which the body of a `const fn` is not.
macro_rules! zone_endpoint {
    ($index:literal) => {
        Endpoint::new(
            zone_endpoint_id($index),
            // Per the Device Library, a bridged endpoint carries the Bridged
            // Node device type alongside that of the thing being bridged.
            devices!(DEV_TYPE_BRIDGED_NODE, DEV_TYPE_THERMOSTAT),
            clusters!(
                desc::DescHandler::CLUSTER,
                BDBI_CLUSTER,
                identify::CLUSTER,
                THERMOSTAT_CLUSTER
            ),
        )
    };
}

/// Chains the three stateful clusters of each listed zone onto `$handler`.
///
/// A macro because `chain` is type-level: each call returns a distinct
/// `ChainedHandler<..>` type, so the chain cannot be built by folding over the
/// zone arrays at runtime.
macro_rules! chain_zones {
    ($handler:expr, $identifies:expr, $infos:expr, $thermostats:expr, $($index:literal),+) => {{
        // Indexing the per-zone arrays by literal is not something the
        // compiler rejects when the literal is out of range - it compiles and
        // panics at boot - so check the list covers exactly the zones that
        // exist.
        const _: () = assert!(
            [$($index),+].len() == ZONE_COUNT,
            "`chain_zones!` needs exactly one index per ZONE_NAMES entry"
        );

        $handler
        $(
            .chain(
                EpClMatcher::new(
                    Some(zone_endpoint_id($index)),
                    Some(identify::CLUSTER.id),
                ),
                Async(identify::HandlerAdaptor(&$identifies[$index])),
            )
            .chain(
                EpClMatcher::new(Some(zone_endpoint_id($index)), Some(BDBI_CLUSTER.id)),
                Async(bdbi::HandlerAdaptor(&$infos[$index])),
            )
            .chain(
                EpClMatcher::new(Some(zone_endpoint_id($index)), Some(THERMOSTAT_CLUSTER.id)),
                Async(thermostat::HandlerAdaptor(&$thermostats[$index])),
            )
        )+
    }};
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

/// The amount of memory for allocating all `rs-matter-stack` futures created during
/// the execution of the `run*` methods.
/// This does NOT include the rest of the Matter stack.
///
/// The futures of `rs-matter-stack` created during the execution of the `run*` methods
/// are allocated in a special way using a small bump allocator which results
/// in a much lower memory usage by those.
///
/// If - for your platform - this size is not enough, increase it until
/// the program runs without panics during the stack initialization.
// RP2350 (thumbv8m) has larger stack frames than RP2040 (thumbv6m), so its coex
// futures need a bigger bump arena.
#[cfg(feature = "rp2040")]
const BUMP_SIZE: usize = 30000;
#[cfg(not(feature = "rp2040"))]
const BUMP_SIZE: usize = 40960;

#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

/// How often the commissioning code is re-printed while the device has no fabrics.
const PAIRING_REMINDER: Duration = Duration::from_secs(30);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Necessary `embassy-rp` and `cyw43` initialization boilerplate

    let p = embassy_rp::init(Default::default());

    // Start logging over the USB CDC serial interface, so logs (including the
    // commissioning QR code) are visible on the host without a debug probe.
    spawner.spawn(logger_task(UsbDriver::new(p.USB, UsbIrqs)).unwrap());

    // Whatever the previous run died of, now that the logger is up
    report_last_panic().await;

    // `rs-matter` uses the `x509` crate which (still) needs a few kilos of heap space
    {
        // NimBLE allocates its mbuf/transport pools (~13K with the stock counts) and its GATT
        // registry from this same heap, so it needs a good deal more headroom than `trouble`,
        // which allocates nothing here.
        #[cfg(not(feature = "nimble"))]
        const HEAP_SIZE: usize = 8192;
        #[cfg(feature = "nimble")]
        const HEAP_SIZE: usize = 8192 + 16384;

        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
    }

    info!("Starting...");

    #[cfg(feature = "skip-cyw43-firmware")]
    let (fw, clm, btfw, nvram) = (
        Option::<&Aligned<A4, [u8]>>::None,
        Option::<&Aligned<A4, [u8]>>::None,
        Option::<&Aligned<A4, [u8]>>::None,
        Option::<&Aligned<A4, [u8]>>::None,
    );

    #[cfg(not(feature = "skip-cyw43-firmware"))]
    let (fw, clm, btfw, nvram) = (
        Option::<&Aligned<A4, [u8]>>::Some(aligned_bytes!("../../cyw43-firmware/43439A0.bin")),
        Option::<&Aligned<A4, [u8]>>::Some(aligned_bytes!("../../cyw43-firmware/43439A0_clm.bin")),
        Option::<&Aligned<A4, [u8]>>::Some(aligned_bytes!("../../cyw43-firmware/43439A0_btfw.bin")),
        Option::<&Aligned<A4, [u8]>>::Some(aligned_bytes!("../../cyw43-firmware/nvram_rp2040.bin")),
    );

    // Statically allocate the Matter stack.
    // For MCUs, it is best to allocate it statically, so as to avoid program stack blowups (its memory footprint is ~ 35 to 50KB).
    // It is also (currently) a mandatory requirement when the wireless stack variation is used.
    let stack = mk_static!(EmbassyWifiMatterStack<BUMP_SIZE, ()>).init_with(
        EmbassyWifiMatterStack::init(&DEV_DET, TEST_DEV_COMM, &TEST_DEV_ATT),
    );

    // Create the crypto provider, using the ROSC RNG peripheral (which is a TRNG) as the source of randomness for a reseeding CSPRNG.
    let crypto = default_crypto(reseeding_csprng(RoscRng, 1000).unwrap(), DAC_PRIVKEY);

    let mut weak_rand = crypto.weak_rand().unwrap();

    // One `Descriptor` handler serves every zone endpoint: `DescHandler` reads
    // the endpoint it is answering for out of the operation context, so it
    // needs no per-endpoint state. The aggregator endpoint gets its own,
    // because its `PartsList` has to enumerate the bridged endpoints instead of
    // being empty.
    let zone_desc = desc::DescHandler::new(Dataver::new_rand(&mut weak_rand));
    let aggregator_desc = desc::DescHandler::new_aggregator(Dataver::new_rand(&mut weak_rand));

    // `Identify`, `BridgedDeviceBasicInformation` and `Thermostat`, on the
    // other hand, all keep per-endpoint state (and a per-endpoint data version),
    // so each zone gets its own instances.
    let identifies: [identify::IdentifyHandler; ZONE_COUNT] =
        core::array::from_fn(|_| identify::IdentifyHandler::new(Dataver::new_rand(&mut weak_rand)));
    let infos: [BridgedZoneInfo; ZONE_COUNT] =
        core::array::from_fn(|i| BridgedZoneInfo::new(Dataver::new_rand(&mut weak_rand), i));
    let thermostats: [ThermostatZone; ZONE_COUNT] =
        core::array::from_fn(|i| ThermostatZone::new(Dataver::new_rand(&mut weak_rand), i));

    // Chain the endpoint clusters.
    //
    // Two things to note. `chain` puts the new handler at the *front* of the
    // chain and the first matching matcher wins, so the aggregator's
    // `Descriptor` has to be chained *after* the catch-all zone one to take
    // precedence for ep1. And every `chain` call produces a new, larger
    // handler type, which is why the per-zone part is a macro rather than a
    // loop over the zone arrays.
    let handler = EmptyHandler
        .chain(
            EpClMatcher::new(None, Some(desc::DescHandler::CLUSTER.id)),
            Async(zone_desc.adapt()),
        )
        .chain(
            EpClMatcher::new(
                Some(AGGREGATOR_ENDPOINT_ID),
                Some(desc::DescHandler::CLUSTER.id),
            ),
            Async(aggregator_desc.adapt()),
        );

    let handler = chain_zones!(handler, identifies, infos, thermostats, 0, 1);

    // Create a KV BLOB store and load any previously saved state of `rs-matter`
    // `SeqMapKvBlobStore` saves to a user-supplied NOR Flash region
    // However, for this demo and for simplicity, we use a dummy KV BLOB store that does nothing
    let mut store = DummyKvBlobStore;
    stack.startup(&crypto, &mut store).await.unwrap();

    let kv = stack.matter().kv(store);

    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but reduces the size of the final future
    //
    // This step can be repeated in that the stack can be stopped and started multiple times, as needed.
    let matter = pin!(stack.run_coex(
        // The Matter stack needs Wifi and BLE
        EmbassyWifi::new(
            RpWifiDriver::new(
                p.PIN_23, p.PIN_25, p.PIN_24, p.PIN_29, p.DMA_CH0, p.DMA_CH1, p.PIO0, Irqs, fw,
                clm, btfw, nvram,
            ),
            crypto.rand().unwrap(),
            true, // Use a random BLE address
            stack,
        ),
        // The crypto provider
        &crypto,
        // Our `AsyncHandler` + `AsyncMetadata` impl
        (NODE, handler),
        // The Matter stack needs a blob store to store its state
        kv,
        // No user future to run
        (),
    ));

    // Keep re-printing the commissioning code, so that attaching the serial
    // terminal late still gets you something to commission with.
    let reminder = pin!(pairing_reminder(
        stack.matter(),
        stack.network().discovery_capabilities(),
        PAIRING_REMINDER,
    ));

    // Run Matter
    embassy_futures::select::select(matter, reminder)
        .coalesce()
        .await
        .unwrap();
}

/// Basic information for the node.
///
/// The stock `TEST_DEV_DET` leaves `device_type` unset and names itself
/// "MyTest", so this overrides both while keeping the test vendor/product IDs -
/// those have to stay, because `TEST_DEV_ATT` attests to exactly that pair.
///
/// `device_name` and `device_type` are not real Basic Information attributes:
/// they only ever reach a commissioner through the `DN` and `DT` keys of the
/// `_matterc._udp` mDNS record (and `device_type` additionally as the `_T<dt>`
/// PTR subtype). A commissioner that finds the device over BLE sees neither -
/// the Matter BLE service data has room for the discriminator and the
/// vendor/product IDs and nothing else - so these matter for on-network
/// commissioning and for the window in concurrent commissioning after the
/// device has joined Wifi. `product_name` and `vendor_name` *are* real
/// attributes, and are what a controller reads back once it is in.
///
/// The advertised device type is the node's primary one, which for a bridge is
/// the Aggregator on ep1 rather than anything on a bridged endpoint.
const DEV_DET: BasicInfoConfig = BasicInfoConfig {
    device_name: "Heating Zones",
    device_type: Some(DEV_TYPE_AGGREGATOR.dtype),
    product_name: "Heating Zone Bridge",
    ..TEST_DEV_DET
};

/// The names the bridge suggests to the ecosystem, via `NodeLabel`.
///
/// This list is what defines how many zones there are. Nothing in `rs-matter`
/// or `rs-matter-stack` caps the endpoint count, and a controller's wildcard
/// subscription covers all of them at once - but three *other* places have to
/// agree with this one, because the handler chain and the node metadata are
/// both built at compile time and cannot be looped over:
///
/// * `ZONE_UNIQUE_IDS`, below
/// * the `zone_endpoint!` list in `NODE`
/// * the index list passed to `chain_zones!` in `main`
///
/// All three are checked against this list by `const` assertions, so getting
/// one wrong is a compile error rather than a panic at boot.
const ZONE_NAMES: &[&str] = &["Living Room", "Bathroom"];

/// The number of simulated heating zones, derived from [`ZONE_NAMES`].
const ZONE_COUNT: usize = ZONE_NAMES.len();

/// Endpoint 0 (the root endpoint) always runs the hidden Matter system
/// clusters, so the aggregator gets ID=1 and the zones follow it.
const AGGREGATOR_ENDPOINT_ID: u16 = 1;

/// The endpoint ID of zone `index`.
const fn zone_endpoint_id(index: usize) -> u16 {
    AGGREGATOR_ENDPOINT_ID + 1 + index as u16
}

/// The Matter Thermostat device type (Matter Device Library).
///
/// `rs-matter` ships constants for the device types it has example clusters
/// for; Thermostat is not one of them, so declare it here. It mandates exactly
/// two clusters - Identify and Thermostat - both of which this endpoint has.
const DEV_TYPE_THERMOSTAT: DeviceType = DeviceType {
    dtype: 0x0301,
    drev: 4,
};

/// The Matter node: an aggregator plus one bridged Thermostat per zone.
const NODE: Node = Node {
    endpoints: &[
        EmbassyWifiMatterStack::<0, ()>::root_endpoint(),
        Endpoint::new(
            AGGREGATOR_ENDPOINT_ID,
            devices!(DEV_TYPE_AGGREGATOR),
            clusters!(desc::DescHandler::CLUSTER),
        ),
        zone_endpoint!(0),
        zone_endpoint!(1),
    ],
};

// The root endpoint and the aggregator, plus one endpoint per zone.
const _: () = assert!(
    NODE.endpoints.len() == ZONE_COUNT + 2,
    "NODE needs exactly one `zone_endpoint!` per ZONE_NAMES entry"
);

//
// BridgedDeviceBasicInformation
//

/// Only `Reachable` is mandatory; `NodeLabel` is what actually earns the bridge
/// topology its keep (a name per zone), and `UniqueID` gives each zone a stable
/// identity across reboots so controllers do not treat it as a new device.
const BDBI_CLUSTER: Cluster<'static> = bdbi::FULL_CLUSTER.with_attrs(with!(
    required;
    bdbi::AttributeId::NodeLabel | bdbi::AttributeId::UniqueID
));

/// The `BridgedDeviceBasicInformation` cluster for one zone.
struct BridgedZoneInfo {
    dataver: Dataver,
    /// Writable, so a controller can rename the zone. Bounded at 32 bytes,
    /// which is the `NodeLabel` constraint from the spec.
    node_label: Mutex<RefCell<String<32>>>,
    unique_id: &'static str,
}

impl BridgedZoneInfo {
    fn new(dataver: Dataver, index: usize) -> Self {
        Self {
            dataver,
            node_label: Mutex::new(RefCell::new(String::try_from(ZONE_NAMES[index]).unwrap())),
            // Static, because the zones are: in a real device this would be
            // the sensor's BLE address or its serial number.
            unique_id: ZONE_UNIQUE_IDS[index],
        }
    }
}

/// Stable per-zone identities. Static strings so that `unique_id` needs no
/// formatting buffer.
const ZONE_UNIQUE_IDS: &[&str] = &["zone-1", "zone-2"];

const _: () = assert!(
    ZONE_UNIQUE_IDS.len() == ZONE_COUNT,
    "ZONE_UNIQUE_IDS must have one entry per ZONE_NAMES entry"
);

impl bdbi::ClusterHandler for BridgedZoneInfo {
    const CLUSTER: Cluster<'static> = BDBI_CLUSTER;

    fn dataver(&self) -> u32 {
        self.dataver.get()
    }

    fn dataver_changed(&self) {
        self.dataver.changed();
    }

    fn reachable(&self, _ctx: impl ReadContext) -> Result<bool, Error> {
        // The simulated sensors never go away. A real bridge would return
        // whether the zone's BLE link is up, and `notify_attr_changed` on
        // `Reachable` whenever that changed.
        Ok(true)
    }

    fn node_label<P: TLVBuilderParent>(
        &self,
        _ctx: impl ReadContext,
        builder: Utf8StrBuilder<P>,
    ) -> Result<P, Error> {
        self.node_label
            .lock(|label| builder.set(label.borrow().as_str()))
    }

    fn set_node_label(&self, ctx: impl WriteContext, value: Utf8Str<'_>) -> Result<(), Error> {
        let label = String::try_from(value).map_err(|_| Error::from(ErrorCode::ConstraintError))?;

        self.node_label.lock(|l| *l.borrow_mut() = label);
        ctx.notify_changed();

        Ok(())
    }

    fn unique_id<P: TLVBuilderParent>(
        &self,
        _ctx: impl ReadContext,
        builder: Utf8StrBuilder<P>,
    ) -> Result<P, Error> {
        builder.set(self.unique_id)
    }

    fn handle_keep_active(
        &self,
        _ctx: impl InvokeContext,
        _request: bdbi::KeepActiveRequest<'_>,
    ) -> Result<(), Error> {
        // Only meaningful for bridged ICDs (sleepy devices), which we do not
        // claim to be - hence the command is not in `BDBI_CLUSTER` either.
        Err(ErrorCode::CommandNotFound.into())
    }
}

//
// Thermostat
//

/// Heat-only: the `HEATING` feature, and on top of the cluster's mandatory
/// attributes the three that the feature brings with it.
///
/// `SetpointRaiseLower` is the cluster's only mandatory command; the schedule,
/// preset and atomic-write commands all belong to features we do not claim, so
/// they stay out of the metadata (and their handlers below just say so).
const THERMOSTAT_CLUSTER: Cluster<'static> = thermostat::FULL_CLUSTER
    .with_features(thermostat::Feature::HEATING.bits())
    .with_attrs(with!(
        required;
        thermostat::AttributeId::OccupiedHeatingSetpoint
            | thermostat::AttributeId::AbsMinHeatSetpointLimit
            | thermostat::AttributeId::AbsMaxHeatSetpointLimit
            | thermostat::AttributeId::ThermostatRunningState
    ))
    .with_cmds(with!(thermostat::CommandId::SetpointRaiseLower));

/// Matter carries temperatures as hundredths of a degree Celsius.
const fn celsius(whole: i16, hundredths: i16) -> i16 {
    whole * 100 + hundredths
}

/// The setpoint range we accept, reported as `AbsMinHeatSetpointLimit` /
/// `AbsMaxHeatSetpointLimit`.
///
/// These are what a controller reads to build its setpoint dial: the spec's
/// user-configurable `Min`/`MaxHeatSetpointLimit` pair is optional and not
/// advertised here, so the absolute limits are the effective range. 15 °C
/// rather than the spec's 7 °C default because nothing sensible asks a central
/// heating zone for less.
const ABS_MIN_HEAT_SETPOINT: i16 = celsius(15, 0);
const ABS_MAX_HEAT_SETPOINT: i16 = celsius(30, 0);

/// Where a zone drifts towards when its system mode is `Off`. Below
/// `ABS_MIN_HEAT_SETPOINT`, so switching a zone off is always visible as the
/// temperature falling away from any setpoint it could have been given.
const AMBIENT_TEMPERATURE: i16 = celsius(12, 0);

/// How fast the simulation runs: one step of `SIMULATION_STEP` every
/// `SIMULATION_TICK`. Slow enough to look like a room, fast enough that you do
/// not have to wait around after writing a setpoint.
const SIMULATION_TICK: Duration = Duration::from_secs(2);
const SIMULATION_STEP: i16 = celsius(0, 10);

struct ZoneState {
    /// `LocalTemperature`, in 0.01 °C.
    local_temperature: i16,
    /// `OccupiedHeatingSetpoint`, in 0.01 °C.
    heating_setpoint: i16,
    system_mode: thermostat::SystemModeEnum,
    /// The heat-relay state last reported to controllers.
    ///
    /// Reads of `ThermostatRunningState` are computed on demand and so are
    /// always current; this exists only so the simulation task can notice a
    /// flip and report it - including a flip caused by a setpoint or
    /// system-mode *write*, which it would otherwise miss by comparing only
    /// against the state at the top of its own step.
    reported_heating: bool,
}

impl ZoneState {
    /// Whether the zone is calling for heat, i.e. the `HEAT` bit of
    /// `ThermostatRunningState`.
    fn is_heating(&self) -> bool {
        matches!(self.system_mode, thermostat::SystemModeEnum::Heat)
            && self.local_temperature < self.heating_setpoint
    }
}

/// What one simulation step changed, and hence what needs reporting.
#[derive(Default)]
struct StepOutcome {
    temperature_changed: bool,
    running_state_changed: bool,
}

/// The `Thermostat` cluster for one simulated zone.
struct ThermostatZone {
    dataver: Dataver,
    /// Captured at construction: the run task needs it to address its
    /// attribute-changed notifications, and `HandlerContext` does not carry an
    /// endpoint.
    endpoint_id: u16,
    state: Mutex<RefCell<ZoneState>>,
}

impl ThermostatZone {
    fn new(dataver: Dataver, index: usize) -> Self {
        Self {
            dataver,
            endpoint_id: zone_endpoint_id(index),
            state: Mutex::new(RefCell::new(ZoneState {
                // Stagger the zones so they are easy to tell apart in a
                // controller before anything has been written to them. The
                // half-degree offset keeps every starting temperature distinct
                // from the starting setpoint below, so that a value seen in a
                // controller is never ambiguous between the two.
                local_temperature: celsius(16, 50) + celsius(1, 0) * index as i16,
                heating_setpoint: celsius(21, 0),
                system_mode: thermostat::SystemModeEnum::Heat,
                // Every zone starts below its setpoint in `Heat`, so every
                // zone starts out calling for heat.
                reported_heating: true,
            })),
        }
    }

    /// Move the simulated temperature one step towards where it should be
    /// heading, and report what changed.
    fn simulate_step(&self) -> StepOutcome {
        self.state.lock(|state| {
            let mut state = state.borrow_mut();

            let target = match state.system_mode {
                thermostat::SystemModeEnum::Heat => state.heating_setpoint,
                _ => AMBIENT_TEMPERATURE,
            };

            let delta = target - state.local_temperature;

            if delta != 0 {
                // Never overshoot: the last step is however much is left.
                state.local_temperature += delta.clamp(-SIMULATION_STEP, SIMULATION_STEP);
            }

            let heating = state.is_heating();
            let running_state_changed = heating != state.reported_heating;
            state.reported_heating = heating;

            StepOutcome {
                temperature_changed: delta != 0,
                running_state_changed,
            }
        })
    }

    /// Apply a new heating setpoint, clamped to the advertised limits.
    fn set_heating_setpoint(&self, setpoint: i16) {
        let setpoint = setpoint.clamp(ABS_MIN_HEAT_SETPOINT, ABS_MAX_HEAT_SETPOINT);

        self.state
            .lock(|state| state.borrow_mut().heating_setpoint = setpoint);
    }
}

impl thermostat::ClusterHandler for ThermostatZone {
    const CLUSTER: Cluster<'static> = THERMOSTAT_CLUSTER;

    fn dataver(&self) -> u32 {
        self.dataver.get()
    }

    fn dataver_changed(&self) {
        self.dataver.changed();
    }

    fn local_temperature(&self, _ctx: impl ReadContext) -> Result<Nullable<i16>, Error> {
        // `Nullable::none()` would be the honest answer for a zone whose sensor
        // is unreachable.
        Ok(Nullable::some(
            self.state.lock(|state| state.borrow().local_temperature),
        ))
    }

    fn abs_min_heat_setpoint_limit(&self, _ctx: impl ReadContext) -> Result<i16, Error> {
        Ok(ABS_MIN_HEAT_SETPOINT)
    }

    fn abs_max_heat_setpoint_limit(&self, _ctx: impl ReadContext) -> Result<i16, Error> {
        Ok(ABS_MAX_HEAT_SETPOINT)
    }

    fn occupied_heating_setpoint(&self, _ctx: impl ReadContext) -> Result<i16, Error> {
        Ok(self.state.lock(|state| state.borrow().heating_setpoint))
    }

    fn set_occupied_heating_setpoint(
        &self,
        ctx: impl WriteContext,
        value: i16,
    ) -> Result<(), Error> {
        // Out-of-range writes are rejected rather than clamped, per the
        // Application Cluster spec.
        if !(ABS_MIN_HEAT_SETPOINT..=ABS_MAX_HEAT_SETPOINT).contains(&value) {
            return Err(ErrorCode::ConstraintError.into());
        }

        self.set_heating_setpoint(value);
        ctx.notify_changed();

        Ok(())
    }

    fn control_sequence_of_operation(
        &self,
        _ctx: impl ReadContext,
    ) -> Result<thermostat::ControlSequenceOfOperationEnum, Error> {
        Ok(thermostat::ControlSequenceOfOperationEnum::HeatingOnly)
    }

    fn set_control_sequence_of_operation(
        &self,
        _ctx: impl WriteContext,
        value: thermostat::ControlSequenceOfOperationEnum,
    ) -> Result<(), Error> {
        // The attribute is writable, but with only the `HEATING` feature there
        // is exactly one legal value.
        if matches!(
            value,
            thermostat::ControlSequenceOfOperationEnum::HeatingOnly
        ) {
            Ok(())
        } else {
            Err(ErrorCode::ConstraintError.into())
        }
    }

    fn thermostat_running_state(
        &self,
        _ctx: impl ReadContext,
    ) -> Result<thermostat::RelayStateBitmap, Error> {
        // Heat-only, single stage: the `HEAT` bit is the whole story. A real
        // controller would report the actual relay here, and would also want
        // `PIHeatingDemand` if it modulates rather than bangs on and off.
        Ok(if self.state.lock(|state| state.borrow().is_heating()) {
            thermostat::RelayStateBitmap::HEAT
        } else {
            thermostat::RelayStateBitmap::empty()
        })
    }

    fn system_mode(&self, _ctx: impl ReadContext) -> Result<thermostat::SystemModeEnum, Error> {
        Ok(self.state.lock(|state| state.borrow().system_mode))
    }

    fn set_system_mode(
        &self,
        ctx: impl WriteContext,
        value: thermostat::SystemModeEnum,
    ) -> Result<(), Error> {
        // A heat-only thermostat supports `Off` and `Heat` and nothing else.
        if !matches!(
            value,
            thermostat::SystemModeEnum::Off | thermostat::SystemModeEnum::Heat
        ) {
            return Err(ErrorCode::ConstraintError.into());
        }

        self.state
            .lock(|state| state.borrow_mut().system_mode = value);
        ctx.notify_changed();

        Ok(())
    }

    fn handle_setpoint_raise_lower(
        &self,
        ctx: impl InvokeContext,
        request: thermostat::SetpointRaiseLowerRequest<'_>,
    ) -> Result<(), Error> {
        // `Cool` and `Both` would touch a cooling setpoint we do not have.
        if matches!(
            request.mode()?,
            thermostat::SetpointRaiseLowerModeEnum::Cool
        ) {
            return Err(ErrorCode::ConstraintError.into());
        }

        // `amount` is in steps of 0.1 °C, per the Application Cluster spec.
        let delta = request.amount()? as i16 * celsius(0, 10);
        let current = self.state.lock(|state| state.borrow().heating_setpoint);

        self.set_heating_setpoint(current.saturating_add(delta));

        ctx.notify_own_attr_changed(thermostat::AttributeId::OccupiedHeatingSetpoint as _);

        Ok(())
    }

    async fn run(&self, ctx: impl HandlerContext) -> Result<(), Error> {
        loop {
            Timer::after(SIMULATION_TICK).await;

            let outcome = self.simulate_step();

            if outcome.temperature_changed {
                ctx.notify_attr_changed(
                    self.endpoint_id,
                    Self::CLUSTER.id,
                    thermostat::AttributeId::LocalTemperature as _,
                );
            }

            // A write can flip this too; picking it up here rather than in the
            // write handlers costs at most one tick of reporting latency and
            // keeps the flip detection in one place.
            if outcome.running_state_changed {
                ctx.notify_attr_changed(
                    self.endpoint_id,
                    Self::CLUSTER.id,
                    thermostat::AttributeId::ThermostatRunningState as _,
                );
            }
        }
    }

    //
    // Everything below belongs to Thermostat features this endpoint does not
    // claim (schedules, presets, atomic writes). The commands are not in
    // `THERMOSTAT_CLUSTER`, so a spec-compliant controller will never send
    // them; the trait requires the methods regardless.
    //

    fn handle_set_weekly_schedule(
        &self,
        _ctx: impl InvokeContext,
        _request: thermostat::SetWeeklyScheduleRequest<'_>,
    ) -> Result<(), Error> {
        Err(ErrorCode::CommandNotFound.into())
    }

    fn handle_get_weekly_schedule<P: TLVBuilderParent>(
        &self,
        _ctx: impl InvokeContext,
        _request: thermostat::GetWeeklyScheduleRequest<'_>,
        _response: thermostat::GetWeeklyScheduleResponseBuilder<P>,
    ) -> Result<P, Error> {
        Err(ErrorCode::CommandNotFound.into())
    }

    fn handle_clear_weekly_schedule(&self, _ctx: impl InvokeContext) -> Result<(), Error> {
        Err(ErrorCode::CommandNotFound.into())
    }

    fn handle_set_active_schedule_request(
        &self,
        _ctx: impl InvokeContext,
        _request: thermostat::SetActiveScheduleRequestRequest<'_>,
    ) -> Result<(), Error> {
        Err(ErrorCode::CommandNotFound.into())
    }

    fn handle_set_active_preset_request(
        &self,
        _ctx: impl InvokeContext,
        _request: thermostat::SetActivePresetRequestRequest<'_>,
    ) -> Result<(), Error> {
        Err(ErrorCode::CommandNotFound.into())
    }

    fn handle_add_thermostat_suggestion<P: TLVBuilderParent>(
        &self,
        _ctx: impl InvokeContext,
        _request: thermostat::AddThermostatSuggestionRequest<'_>,
        _response: thermostat::AddThermostatSuggestionResponseBuilder<P>,
    ) -> Result<P, Error> {
        Err(ErrorCode::CommandNotFound.into())
    }

    fn handle_remove_thermostat_suggestion(
        &self,
        _ctx: impl InvokeContext,
        _request: thermostat::RemoveThermostatSuggestionRequest<'_>,
    ) -> Result<(), Error> {
        Err(ErrorCode::CommandNotFound.into())
    }

    fn handle_atomic_request<P: TLVBuilderParent>(
        &self,
        _ctx: impl InvokeContext,
        _request: thermostat::AtomicRequestRequest<'_>,
        _response: thermostat::AtomicResponseBuilder<P>,
    ) -> Result<P, Error> {
        Err(ErrorCode::CommandNotFound.into())
    }
}
