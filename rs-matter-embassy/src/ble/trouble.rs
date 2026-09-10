//! BLE: `TroubleBtpGattPeripheral` - an implementation of the `GattPeripheral` trait from
//! `rs-matter`, on top of the pure-Rust `trouble-host` BLE stack.
//!
//! Two flavours, differing only in who owns the `trouble-host` stack:
//!
//! * [`TroubleBtpGattPeripheral`] builds and owns one. This is what you want
//!   when Matter is the only thing using the BLE controller.
//! * [`TroubleBtpExternalGattPeripheral`] borrows a `Peripheral` handle from a
//!   stack the caller built. A single controller can back only one
//!   `trouble-host` stack, so a device that *also* needs the BLE central role -
//!   a bridge polling its own BLE sensors, say - cannot let Matter build a
//!   second one. Such a device builds one multirole stack itself and hands
//!   Matter the peripheral half.
//!
//! Both drive the same service implementation ([`run_btp_gatt_service`]).

#![allow(clippy::useless_conversion)] // https://github.com/embassy-rs/trouble/issues/248
#![allow(clippy::needless_borrows_for_generic_args)] // In latest trouble-host: ^^^^^^^^ help: change this to: `External`

use core::fmt::Debug;
use core::mem::MaybeUninit;
use core::ops::Deref;
use core::pin::pin;

use embassy_futures::select::select;

use rs_matter_stack::ble::GattPeripheral;
use rs_matter_stack::matter::crypto::RngCore;
use rs_matter_stack::matter::error::{Error, ErrorCode};
use rs_matter_stack::matter::transport::network::btp::{
    AdvData, Btp, C1_CHARACTERISTIC_UUID, C2_CHARACTERISTIC_UUID, MATTER_BLE_SERVICE_UUID16,
};
use rs_matter_stack::matter::transport::network::BtAddr;
use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::matter::utils::select::Coalesce;
use rs_matter_stack::matter::utils::storage::Vec;
use rs_matter_stack::matter::utils::sync::{IfMutex, Notification};

use trouble_host::att::{AttClient, AttReq, AttRsp};
use trouble_host::prelude::*;
use trouble_host::{self, BleHostError, HostResources};

use super::ControllerRef;
use crate::fmt::Bytes;

/// The `bt-hci` controller contract of the selected BLE backend.
pub use trouble_host::Controller;

/// The BTP GATT context of the selected BLE backend.
pub type BtpGattContext = TroubleBtpGattContext;

/// The BTP GATT peripheral of the selected BLE backend.
pub type BtpGattPeripheral<'a, R, C> = TroubleBtpGattPeripheral<'a, R, C>;

/// The BTP GATT peripheral of the selected BLE backend, for a caller-owned stack.
pub type BtpExternalGattPeripheral<'a, 'd, C> = TroubleBtpExternalGattPeripheral<'a, 'd, C>;

/// The indication-buffer context of the selected BLE backend.
pub type BtpGattIndContext = TroubleBtpGattIndContext;

const MAX_CONNECTIONS: usize = 1;

/// The largest BTP frame the service will assemble, and hence the required size
/// of the indication buffer passed to [`run_btp_gatt_service`].
pub const MAX_MTU_SIZE: usize = DefaultPacketPool::MTU;
const MAX_CHANNELS: usize = 2;
const ADV_SETS: usize = 1;

pub type GPHostResources =
    HostResources<DefaultPacketPool, MAX_CONNECTIONS, MAX_CHANNELS, ADV_SETS>;

type External = [u8; 0];

// GATT Server definition
#[gatt_server]
struct Server {
    matter_service: MatterService,
}

/// Matter service
#[gatt_service(uuid = MATTER_BLE_SERVICE_UUID16)]
struct MatterService {
    #[characteristic(uuid = C1_CHARACTERISTIC_UUID, write)]
    c1: External,
    #[characteristic(uuid = C2_CHARACTERISTIC_UUID, write, indicate)]
    c2: External,
}

struct TroubleBtpResources {
    resources: GPHostResources,
    ind_buf: Vec<u8, MAX_MTU_SIZE>,
}

impl TroubleBtpResources {
    const fn new() -> Self {
        Self {
            resources: GPHostResources::new(),
            ind_buf: Vec::new(),
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            // Note: below will break if `HostResources` stops being a bunch of `MaybeUninit`s
            resources: unsafe { MaybeUninit::<GPHostResources>::uninit().assume_init() },
            ind_buf <- Vec::init(),
        })
    }
}

/// The state of the `TroubleBtpGattPeripheral` struct.
/// Isolated as a separate struct to allow for `const fn` construction
/// and static allocation.
pub struct TroubleBtpGattContext {
    resources: IfMutex<TroubleBtpResources>,
}

impl TroubleBtpGattContext {
    /// Create a new instance.
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            resources: IfMutex::new(TroubleBtpResources::new()),
        }
    }

    /// Return an in-place initializer for the type.
    pub fn init() -> impl Init<Self> {
        init!(Self {
            resources <- IfMutex::init(TroubleBtpResources::init()),
        })
    }

    // pub(crate) fn reset(&self) -> Result<(), ()> {
    //     unwrap!(self.ind
    //         .try_lock()
    //         .map(|mut ind| {
    //             ind.data.clear();
    //         })); // TODO

    //     Ok(())
    // }
}

impl Default for TroubleBtpGattContext {
    // TODO
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    fn default() -> Self {
        Self::new()
    }
}

/// A GATT peripheral implementation for the BTP protocol in `rs-matter` via `trouble-host`.
/// Implements the `GattPeripheral` trait.
pub struct TroubleBtpGattPeripheral<'a, R, C>
where
    R: RngCore + Copy,
    C: Controller,
{
    // TODO: Ideally this should be the controller itself, but this is not possible
    // until `bt-hci` is updated with `impl<C: Controller>` Controller for &C {}`
    ble_ctl: C,
    rand: Option<R>,
    context: &'a TroubleBtpGattContext,
}

impl<'a, R, C> TroubleBtpGattPeripheral<'a, R, C>
where
    R: RngCore + Copy,
    C: Controller,
{
    /// Create a new instance.
    ///
    /// Creation might fail if the GATT context cannot be reset, so user should ensure
    /// that there are no other GATT peripherals running before calling this function.
    pub const fn new(ble_ctl: C, rand: Option<R>, context: &'a TroubleBtpGattContext) -> Self {
        Self {
            ble_ctl,
            rand,
            context,
        }
    }

    /// Run the GATT peripheral.
    pub async fn run(
        &mut self,
        btp: &Btp,
        service_name: &str,
        service_adv_data: &AdvData,
    ) -> Result<(), Error> {
        info!("Starting advertising and GATT service");

        let mut resources = self.context.resources.lock().await;
        let resources = &mut *resources;

        unwrap!(resources.ind_buf.resize_default(MAX_MTU_SIZE));

        let ind_buf = &mut resources.ind_buf;
        let resources = &mut resources.resources;

        let controller = ControllerRef::new(&self.ble_ctl);

        let stack = trouble_host::new(controller, resources);

        let stack = if let Some(mut rand) = self.rand {
            // Generate a valid BLE Static Random Address
            // - Two most significant bits must be 11 (static random address)
            // - Lower 46 bits must contain at least one 0 and one 1
            let address: [u8; 6] = loop {
                let addr = rand.next_u64() & 0x3f_ff_ff_ff_ff_ff;
                if addr != 0 && addr != 0x3f_ff_ff_ff_ff_ff {
                    // A BLE BD_ADDR is little-endian, so its most-significant byte -- the
                    // one that must carry the 0b11 static-random type bits -- is the last of
                    // the six. Emit little-endian so the 0xc0 lands on address[5].
                    break (addr | 0xc0_00_00_00_00_00).to_le_bytes()[..6]
                        .try_into()
                        .unwrap();
                }
            };

            info!("Random GATT address = {:?}", address);

            stack.set_random_address(Address::random(address))
        } else {
            stack
        };

        let stack = stack.build();

        let runner = stack.runner();
        let mut peripheral = stack.peripheral();

        let server = new_server()?;

        let mut ble_task = pin!(run_ble(runner));
        let mut peripheral_task = pin!(run_peripheral(
            btp,
            service_name,
            service_adv_data,
            &server,
            &mut peripheral,
            ind_buf
        ));

        select(&mut ble_task, &mut peripheral_task)
            .coalesce()
            .await?;

        Ok(())
    }
}

async fn run_ble(mut runner: Runner<'_, impl Controller, impl PacketPool>) -> Result<(), Error> {
    loop {
        runner.run().await.map_err(to_matter_err)?;
    }
}

async fn run_peripheral(
    btp: &Btp,
    service_name: &str,
    service_adv_data: &AdvData,
    server: &Server<'_>,
    peripheral: &mut Peripheral<'_, impl Controller, DefaultPacketPool>,
    ind_buf: &mut [u8],
) -> Result<(), Error> {
    loop {
        let conn = advertise(service_name, service_adv_data, peripheral)
            .await
            .map_err(to_matter_err)?;

        let conn = conn
            .with_attribute_server(server.deref())
            .map_err(to_matter_err)?;

        btp.reset();

        let ind_ack = Notification::new();

        let events = handle_events(server, &conn, &ind_ack, btp);
        let indications = handle_indications(server, &conn, ind_buf, &ind_ack, btp);

        select(events, indications).coalesce().await?;
    }
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
async fn advertise<'p, CC: Controller>(
    service_name: &str,
    service_adv_data: &AdvData,
    peripheral: &mut Peripheral<'p, CC, DefaultPacketPool>,
) -> Result<Connection<'p, DefaultPacketPool>, BleHostError<CC::Error>> {
    let service_adv_enc_data = service_adv_data
        .service_payload_iter()
        .collect::<Vec<_, 8>>();

    let adv_data = [
        AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
        AdStructure::ServiceData16 {
            uuid: MATTER_BLE_SERVICE_UUID16.to_le_bytes(),
            data: &service_adv_enc_data,
        },
        AdStructure::CompleteLocalName(service_name.as_bytes()),
    ];

    let mut adv_enc_data = [0; 31];
    let len = AdStructure::encode_slice(&adv_data, &mut adv_enc_data)?;

    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_enc_data[..len],
                scan_data: &[],
            },
        )
        .await?;

    info!("GATT: Advertising");

    let conn = advertiser.accept().await?;

    info!("GATT: Connection established");

    Ok(conn)
}

/// Stream events until the connection closes or the other peer unsibscribes from char C2.
async fn handle_events(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    ind_ack: &Notification,
    btp: &Btp,
) -> Result<(), Error> {
    fn to_bt_addr(addr: &BdAddr) -> BtAddr {
        let raw = addr.raw();
        BtAddr([raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]])
    }

    let mut subscribed = false;

    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("GATT: Disconnect: {:?}", reason);
                break;
            }
            GattConnectionEvent::Gatt { event } => match event.payload().incoming() {
                AttClient::Request(AttReq::Write {
                    handle,
                    data: bytes,
                }) => {
                    if handle == server.matter_service.c1.handle {
                        trace!(
                            "GATT: C1 Write {} len {} / MTU {}",
                            Bytes(bytes),
                            bytes.len(),
                            conn.raw().att_mtu()
                        );

                        btp.process_incoming(
                            Some(conn.raw().att_mtu()),
                            to_bt_addr(&conn.raw().peer_address().addr),
                            bytes,
                        )
                        .map_err(to_matter_err)?;

                        write_reply(event).await?;
                    } else if Some(handle) == server.matter_service.c2.cccd_handle {
                        let subscription_req = bytes[0] != 0;

                        trace!("GATT: Write to C2 CCC descriptor: {:?}", bytes);

                        // NOTE: We MUST let the attribute server process the CCCD write
                        // (via `accept`) rather than replying to it ourselves, so that
                        // `trouble-host` records the subscription in its own CCCD table.
                        // `Characteristic::indicate` in `handle_indications` consults that
                        // table (`should_indicate`) and silently drops the indication if the
                        // peer is not registered as subscribed there.
                        accept(event).await?;

                        if subscription_req {
                            if !subscribed {
                                info!("GATT: Peer subscribed");
                                subscribed = true;
                                ind_ack.notify();
                            }
                        } else if subscribed {
                            info!("GATT: Peer unsubscribed");
                            break;
                        }
                    } else {
                        accept(event).await?;
                    }
                }
                _ => accept(event).await?,
            },
            _ => (),
        }
    }

    info!("GATT: Events task finished");

    Ok(())
}

/// Handle outgoing data from Btp as indications
async fn handle_indications(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    ind_buf: &mut [u8],
    ind_ack: &Notification,
    btp: &Btp,
) -> Result<(), Error> {
    // Wait until `handle_events` indicates to us
    // that the peer did subscribe to char C2
    ind_ack.wait().await;

    loop {
        let len = btp.process_outgoing(Some(conn.raw().att_mtu()), ind_buf)?;
        if len > 0 {
            let data = &ind_buf[..len];

            // Sends the indication and then blocks until the peer's
            // `HandleValueConfirmation` is received (or the 30s ATT transaction
            // timeout elapses, in which case the connection is torn down).
            //
            // NOTE: As of `trouble-host` commit 50d0f9f, the indication confirmation
            // is consumed internally by `trouble-host` and is NO LONGER surfaced as a
            // `GattConnectionEvent` in `handle_events` - hence waiting for it here,
            // rather than via a separate notification signalled from the events task.
            server
                .matter_service
                .c2
                .indicate_raw(conn, data, false)
                .await
                .map_err(to_matter_err)?;

            trace!("GATT: Indicate {} len {}", Bytes(data), len);
        } else {
            btp.wait_outgoing().await;
        }
    }
}

async fn write_reply(event: GattEvent<'_, '_, DefaultPacketPool>) -> Result<(), Error> {
    event
        .into_payload()
        .reply(AttRsp::Write)
        .await
        .map_err(to_matter_err)
}

async fn accept(event: GattEvent<'_, '_, DefaultPacketPool>) -> Result<(), Error> {
    match event.accept() {
        Ok(reply) => {
            reply.send().await;
        }
        Err(e) => {
            warn!("GATT: Error accepting event: {:?}", e);
        }
    }

    Ok(())
}

/// Build the Matter attribute server (the C1/C2 characteristics of the Matter
/// BTP service).
fn new_server() -> Result<Server<'static>, Error> {
    Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "TrouBLE",                                             // TODO
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE, // TODO
    }))
    .map_err(to_matter_err)
}

/// Serve the Matter BTP GATT service on a caller-supplied `Peripheral`.
///
/// This is the whole of the peripheral's behaviour bar the host itself:
/// advertise, accept a connection, bind the Matter attribute server to it, and
/// pump BTP until the peer disconnects or unsubscribes - then advertise again.
///
/// Note what this function does *not* do: drive the host's `Runner`. The caller
/// owns the stack, so the caller is already polling it;
/// [`TroubleBtpGattPeripheral::run`] builds its own stack and therefore runs
/// both.
///
/// `ind_buf` must be at least [`MAX_MTU_SIZE`] bytes, and is where outgoing BTP
/// frames are assembled before being indicated on C2.
pub async fn run_btp_gatt_service<C: Controller>(
    peripheral: &mut Peripheral<'_, C, DefaultPacketPool>,
    ind_buf: &mut [u8],
    btp: &Btp,
    service_name: &str,
    service_adv_data: &AdvData,
) -> Result<(), Error> {
    let server = new_server()?;

    run_peripheral(
        btp,
        service_name,
        service_adv_data,
        &server,
        peripheral,
        ind_buf,
    )
    .await
}

impl<R, C> GattPeripheral for TroubleBtpGattPeripheral<'_, R, C>
where
    R: RngCore + Copy,
    C: Controller,
{
    async fn run(
        &mut self,
        btp: &Btp,
        service_name: &str,
        adv_data: &AdvData,
    ) -> Result<(), Error> {
        TroubleBtpGattPeripheral::run(self, btp, service_name, adv_data)
            .await
            .map_err(|_| {
                error!("Running TroubleBtpGattPeripheral failed");
                ErrorCode::BtpError
            })?;

        Ok(())
    }
}

/// The state of the [`TroubleBtpExternalGattPeripheral`] struct.
///
/// Just the indication buffer: the rest of what
/// [`TroubleBtpGattContext`] holds is `HostResources`, and a caller supplying
/// its own stack has already allocated those. Isolated as a separate struct to
/// allow for `const fn` construction and static allocation.
pub struct TroubleBtpGattIndContext {
    ind_buf: IfMutex<Vec<u8, MAX_MTU_SIZE>>,
}

impl TroubleBtpGattIndContext {
    /// Create a new instance.
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            ind_buf: IfMutex::new(Vec::new()),
        }
    }

    /// Return an in-place initializer for the type.
    pub fn init() -> impl Init<Self> {
        init!(Self {
            ind_buf <- IfMutex::init(Vec::init()),
        })
    }
}

impl Default for TroubleBtpGattIndContext {
    #[inline(always)]
    fn default() -> Self {
        Self::new()
    }
}

/// A GATT peripheral implementation for the BTP protocol in `rs-matter` that
/// runs on a `trouble-host` stack owned by the caller.
///
/// Use this instead of [`TroubleBtpGattPeripheral`] when something other than
/// Matter also needs the BLE controller. A controller can back only one
/// `trouble-host` stack, so the two roles cannot each build their own; the
/// caller builds one multirole stack, keeps `Stack::runner()` and
/// `Stack::central()` for itself, and hands `Stack::peripheral()` here.
///
/// The caller is responsible for polling the host's `Runner` - this type does
/// not, and cannot, since it does not own the stack.
pub struct TroubleBtpExternalGattPeripheral<'a, 'd, C>
where
    C: Controller,
{
    peripheral: &'a mut Peripheral<'d, C, DefaultPacketPool>,
    context: &'a TroubleBtpGattIndContext,
}

impl<'a, 'd, C> TroubleBtpExternalGattPeripheral<'a, 'd, C>
where
    C: Controller,
{
    /// Create a new instance from the peripheral half of a caller-owned stack.
    pub const fn new(
        peripheral: &'a mut Peripheral<'d, C, DefaultPacketPool>,
        context: &'a TroubleBtpGattIndContext,
    ) -> Self {
        Self {
            peripheral,
            context,
        }
    }

    /// Run the GATT peripheral.
    pub async fn run(
        &mut self,
        btp: &Btp,
        service_name: &str,
        service_adv_data: &AdvData,
    ) -> Result<(), Error> {
        info!("Starting advertising and GATT service on an external BLE host");

        let mut ind_buf = self.context.ind_buf.lock().await;
        unwrap!(ind_buf.resize_default(MAX_MTU_SIZE));

        run_btp_gatt_service(
            self.peripheral,
            &mut ind_buf,
            btp,
            service_name,
            service_adv_data,
        )
        .await
    }
}

impl<C> GattPeripheral for TroubleBtpExternalGattPeripheral<'_, '_, C>
where
    C: Controller,
{
    async fn run(
        &mut self,
        btp: &Btp,
        service_name: &str,
        adv_data: &AdvData,
    ) -> Result<(), Error> {
        TroubleBtpExternalGattPeripheral::run(self, btp, service_name, adv_data)
            .await
            .map_err(|_| {
                error!("Running TroubleBtpExternalGattPeripheral failed");
                ErrorCode::BtpError
            })?;

        Ok(())
    }
}

fn to_matter_err<E: Debug>(err: E) -> Error {
    error!("BLE error: {:?}", debug2format!(err)); // TODO: defmt
    ErrorCode::BtpError.into()
}
