//! Support code shared by the RP examples.
//!
//! All of it exists to make the examples usable on a bare Pico / Pico 2 with
//! nothing but a USB cable - no debug probe:
//!
//! * [`logger_task`] pumps `log` output to the host over USB CDC ACM.
//! * [`report_last_panic`] prints the panic message from the *previous* run.
//!   The USB logger is an async task, so it can no longer flush once we have
//!   panicked; instead the panic handler stashes the message in a chunk of RAM
//!   that survives a reset and reboots, and the next boot logs it.
//! * [`pairing_reminder`] re-prints the commissioning code periodically, so
//!   attaching the serial terminal after the device has already booted still
//!   gets you something to commission with.

#![no_std]

use core::fmt::Write as _;
use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use core::ptr::{addr_of_mut, copy_nonoverlapping, read_volatile, write_volatile};

use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};

use embassy_time::{Duration, Timer};

use log::{info, warn};

use rs_matter_embassy::matter::error::Error;
use rs_matter_embassy::matter::pairing::DiscoveryCapabilities;
use rs_matter_embassy::matter::Matter;

bind_interrupts!(pub struct UsbIrqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

/// The size of the `log` ring-buffer.
///
/// Log records written before the host opens the CDC port accumulate here and
/// are flushed on connect, so this needs to be large enough to hold everything
/// printed during startup - the unicode QR code art in particular. Anything
/// that does not fit is silently dropped.
pub const LOG_RINGBUF_SIZE: usize = 16384;

/// The maximum log level pumped out over USB.
///
/// Worth raising to `Debug` when a controller is not showing what you expect:
/// the IDL-generated cluster code logs every attribute read and write at that
/// level, with the value and the handler's result, so the log then shows
/// exactly which attributes the controller asked for and what it was told.
/// It is a *lot* of output - expect the ring-buffer to drop some of it.
pub const LOG_LEVEL: log::LevelFilter = log::LevelFilter::Info;

/// Pumps the `log` output to the host over the USB CDC ACM serial interface.
#[embassy_executor::task]
pub async fn logger_task(driver: UsbDriver<'static, USB>) {
    embassy_usb_logger::run!(LOG_RINGBUF_SIZE, LOG_LEVEL, driver);
}

/// Re-print the commissioning code every `every` for as long as the device has
/// no fabrics, so that a serial terminal attached late still sees it.
///
/// The manual pairing code is always logged; the QR code payload is
/// best-effort, as printing it borrows the transport's RX buffer, which may be
/// busy with an in-flight commissioning exchange.
///
/// Returns `Result` (and never `Ok`) purely so it can be `Coalesce`d with the
/// Matter stack's own future.
pub async fn pairing_reminder(
    matter: &Matter<'_>,
    caps: DiscoveryCapabilities,
    every: Duration,
) -> Result<(), Error> {
    // The first tick doubles as "we have been up this long without panicking",
    // which is what re-arms the panic handler's auto-reset budget.
    Timer::after(every).await;
    clear_panic_history();

    loop {
        if !matter.has_fabrics() {
            info!(
                "Not commissioned yet. PairingCode: [{}]",
                matter.dev_comm().compute_pretty_pairing_code()
            );

            let _ = matter.print_standard_qr_text(caps);
        }

        Timer::after(every).await;
    }
}

/// Marks [`PanicState`] as initialized. Anything else means we are looking at
/// whatever the RAM happened to contain at power-on.
const PANIC_MAGIC: u32 = 0x504e_4331; // "PNC1"

/// How much of the panic message we keep. Truncated on a char boundary.
const PANIC_MSG_CAP: usize = 480;

/// How many times in a row we are willing to reboot for a panic before giving
/// up and halting. Without this, a panic during startup would reset-loop, and
/// USB would never stay up long enough to read the message off.
const MAX_AUTO_RESETS: u32 = 3;

#[repr(C)]
struct PanicState {
    /// `PANIC_MAGIC` once the rest of the struct is meaningful.
    magic: u32,
    /// Non-zero while a message is waiting to be reported.
    pending: u32,
    /// Consecutive panic reboots, cleared by [`clear_panic_history`].
    resets: u32,
    /// Length of the message in `msg`.
    len: u32,
    msg: [u8; PANIC_MSG_CAP],
}

/// Lives in `.uninit`, which `cortex-m-rt` neither loads nor zeroes at
/// startup - so it survives the reset the panic handler triggers. It does not
/// survive a power cycle, which is why the panic handler reboots rather than
/// halting: the Pico has no reset button, so "halt and let the user reset"
/// would mean a re-plug, and a re-plug loses the message.
#[link_section = ".uninit.PANIC_STATE"]
static mut PANIC_STATE: MaybeUninit<PanicState> = MaybeUninit::uninit();

fn panic_state() -> *mut PanicState {
    unsafe { (*addr_of_mut!(PANIC_STATE)).as_mut_ptr() }
}

/// Stash the panic message where the next boot can find it, then reboot so the
/// USB logger gets a chance to print it.
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let state = panic_state();

    // Volatile throughout: on the first ever boot this memory is genuinely
    // uninitialized, and the magic check is the only thing standing between us
    // and garbage.
    let initialized = unsafe { read_volatile(addr_of_mut!((*state).magic)) } == PANIC_MAGIC;
    let resets = if initialized {
        unsafe { read_volatile(addr_of_mut!((*state).resets)) }
    } else {
        0
    };

    let mut writer = MsgWriter {
        buf: unsafe { addr_of_mut!((*state).msg) }.cast::<u8>(),
        len: 0,
    };
    // `MsgWriter` truncates instead of failing, so this cannot itself panic.
    let _ = write!(writer, "{}", info);

    unsafe {
        write_volatile(addr_of_mut!((*state).len), writer.len as u32);
        write_volatile(addr_of_mut!((*state).resets), resets.saturating_add(1));
        write_volatile(addr_of_mut!((*state).pending), 1);
        write_volatile(addr_of_mut!((*state).magic), PANIC_MAGIC);
    }

    if resets >= MAX_AUTO_RESETS {
        // Panicking every boot. Stop rebooting and let the message stand.
        loop {
            cortex_m::asm::wfe();
        }
    }

    cortex_m::peripheral::SCB::sys_reset()
}

/// Log the panic message left behind by the previous run, if any.
///
/// Call this right after spawning [`logger_task`].
pub fn report_last_panic() {
    let state = panic_state();

    let pending = unsafe {
        read_volatile(addr_of_mut!((*state).magic)) == PANIC_MAGIC
            && read_volatile(addr_of_mut!((*state).pending)) != 0
    };

    if !pending {
        return;
    }

    // Leave `resets` alone - `pairing_reminder` clears it once we have stayed
    // up for a while, which is the only evidence that the panic is not
    // happening on every boot.
    unsafe { write_volatile(addr_of_mut!((*state).pending), 0) };

    // Clamped: a stale magic left in RAM by an unrelated firmware would
    // otherwise let a garbage length walk off the end of the buffer.
    let len = (unsafe { read_volatile(addr_of_mut!((*state).len)) } as usize).min(PANIC_MSG_CAP);
    let resets = unsafe { read_volatile(addr_of_mut!((*state).resets)) };

    let msg = unsafe { core::slice::from_raw_parts(addr_of_mut!((*state).msg).cast::<u8>(), len) };

    // `MsgWriter` only ever appends whole chars, so this is valid UTF-8.
    match core::str::from_utf8(msg) {
        Ok(msg) => warn!("Rebooted after a panic (#{}): {}", resets, msg),
        Err(_) => warn!("Rebooted after a panic (#{}), message corrupted", resets),
    }

    if resets >= MAX_AUTO_RESETS {
        warn!("Panicked {} times in a row; not rebooting again", resets);
    }
}

/// Forget the consecutive-panic count, re-arming the panic handler's reboot
/// budget. Called once we have been up long enough to call the boot a success.
pub fn clear_panic_history() {
    let state = panic_state();

    if unsafe { read_volatile(addr_of_mut!((*state).magic)) } == PANIC_MAGIC {
        unsafe { write_volatile(addr_of_mut!((*state).resets), 0) };
    }
}

/// Appends into the panic message buffer, truncating on a char boundary rather
/// than failing - a panic handler has nowhere to report an error to.
struct MsgWriter {
    buf: *mut u8,
    len: usize,
}

impl core::fmt::Write for MsgWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let mut n = (PANIC_MSG_CAP - self.len).min(s.len());

        while n > 0 && !s.is_char_boundary(n) {
            n -= 1;
        }

        unsafe { copy_nonoverlapping(s.as_ptr(), self.buf.add(self.len), n) };
        self.len += n;

        Ok(())
    }
}
