# Examples

<img src="https://github.com/ivmarkov/rs-matter-embassy/blob/master/examples/acme.jpg" alt="ACME" width="300" height="670">

The examples are tested and _should_ work on the nrf52, rPI Pico W, esp32, esp32s3, esp32c3 and esp32c6.

With that said, it is still early days for all of `rs-matter`, `trouble` (the bare-metal BLE stack in use) 
and `openthread` (the OpenThread Rust wrappers) so you might face issues during the initial commissioning.

Please [report](https://github.com/ivmarkov/rs-matter-embassy/issues) those!

Also, currently the persistance is (temporarily) switched off, so if you stop/restart the MCU, you'll have to go over
the commissioning process again, by first removing your device from your Matter Controller.

## Matter Controller

You need one of:
* **Google**:
  * Google Home/Nest or other Google Matter Controller
  * The Google Home app on your phone
* **Alexa**:
  * Alexa Echo Hub, Echo Dot or other Amazon Matter Controller
  * The Alexa app on your phone
  * Note that Alexa will not work with the Thread examples yet, as no MCU is supported
  * with BLE+Thread coex, and Alexa requires that
* **Apple**:
  * Apple TV or other Apple Matter Controller
  * An iPhone with the Apple Home app
* **Samsung**
  * A recent TV hich can act as a SmartThings controller, or the Samsung / Aeotec SmarthThings standalone hub
  * The SmartThings app on your phone

Once you build and flash the firmware, follow the instructions in the phone app.

To start the commissioning process, all apps will ask you to take a screenshot of the QR code which is printed by the firmware when it starts.
Once you do that, you should see a bunch of logs for the commissioning process.

NOTE: Since the firmware is not certified, the app will warn you about that. Disregard and let it proceed anyway.

Upon successful commissioning, you should end up with a Light device which you can turn on/off, and which will also turn on/off by itself every 5 secs.

## The RP thermostat example

`thermostat_wifi` is the same Wifi + BLE-commissioning assembly as `light_wifi`, but with a more
interesting data model: it exposes two simulated heating zones as a Matter *bridge*.

```text
ep0        Root node       (the hidden Matter system clusters)
ep1        Aggregator      (Descriptor only)
ep2..ep3   Bridged Node + Thermostat
           (Descriptor, BridgedDeviceBasicInformation, Identify, Thermostat)
```

`rs-matter` ships no hand-written Thermostat cluster, but its `build.rs` generates every cluster in
the Matter IDL into `dm::clusters::decl`, so the example implements `decl::thermostat::ClusterHandler`
itself. The zones are heat-only (the `HEATING` feature) and their temperatures drift slowly towards
their heating setpoints, so writing a setpoint from a controller gives visible feedback.

The bridge shape - rather than two bare Thermostat endpoints - is what a real "one MCU, several
remote sensors" device wants: `BridgedDeviceBasicInformation` gives each zone its own `NodeLabel`
(so the zones show up named rather than as "Thermostat 2") and its own `Reachable` flag, which is
how a bridge tells a controller that one of its sensors has gone silent.

## How to build and flash

### rPI Pico and Pico W (RP2040)

(The stock Pico only supports Ethernet using the `light_eth` example and W5500)

```sh
cd rp
cargo +nightly build

# Replace `light_wifi` with `light_eth` or `thermostat_wifi` as needed
probe-rs run --chip rp2040 target/thumbv6m-none-eabi/debug/light_wifi
```

Without a debug probe, hold BOOTSEL while plugging the board in and flash with `picotool` instead -
which is what `cargo run` is configured to do:

```sh
cargo +nightly run --bin light_wifi
```

The examples log over a USB CDC ACM serial interface, so the logs (the commissioning QR code
included) can be read off the board's own USB port with any serial terminal - no probe needed:

```sh
picocom /dev/ttyACM0     # or: screen /dev/ttyACM0, minicom -D /dev/ttyACM0
```

Two things make that usable in practice. The commissioning code is re-printed every 30s for as long
as the device has no fabrics, so attaching the terminal after the board has already booted still
gets you something to commission with. And because the USB logger is an async task - it cannot
flush anything once the firmware has panicked - the panic handler stashes the message in a chunk of
RAM that survives a reset and reboots, so the panic from the previous run is logged on the next
boot. It gives up and halts after three consecutive panics, rather than reset-looping.

### rPI Pico 2 and Pico 2 W (RP2350)

(The stock Pico 2 only supports Ethernet using the `light_eth` example and W5500)

The RP2350 uses a different core and target, so select the matching chip feature
and target (use `rp235xb` instead of `rp235xa` for the QFN-80 RP2350B):

```sh
cd rp
cargo +nightly build --no-default-features --features trouble,rp235xa --target thumbv8m.main-none-eabihf

# Replace `light_wifi` with `light_eth` or `thermostat_wifi` as needed
probe-rs run --chip RP235x target/thumbv8m.main-none-eabihf/debug/light_wifi
```

Or, with no probe, over USB as described for the RP2040 above:

```sh
cargo +nightly run --no-default-features --features trouble,rp235xa \
    --target thumbv8m.main-none-eabihf --bin thermostat_wifi
```

### Espressif MCUs

#### esp32

```sh
# Wifi credentials should be valid only if you plan to run the `light_eth` "ethernet" example.
# The `light_wifi` example gets your Wifi settings from the Matter Controller automatically.
export WIFI_SSID=foo
export WIFI_PASS=bar

cargo install espup
espup update

cd esp
cargo +esp build --target xtensa-esp32-none-elf --no-default-features --features esp32,wifi

# Replace `light_wifi` with `light_eth` below to flash the "Ethernet" example
# Replace `light_wifi` with `light_thread` below to flash the Ethernet example (you'll need an esp32c6 or esp32h2)
espflash flash target/xtensa-esp32-none-elf/debug/light_wifi --baud 1500000
espflash monitor --elf target/xtensa-esp32-none-elf/debug/light_wifi
```

#### esp32s3

```sh
# Wifi credentials should be valid only if you plan to run the `light_eth` "ethernet" example.
# The `light_wifi` example gets your Wifi settings from the Matter Controller automatically.
export WIFI_SSID=foo
export WIFI_PASS=bar

cargo install espup
espup update

cd esp
cargo +esp build --target xtensa-esp32s3-none-elf --no-default-features --features esp32s3,wifi

# Replace `light_wifi` with `light_eth` below to flash the "Ethernet" example
espflash flash target/xtensa-esp32s3-none-elf/debug/light_wifi --baud 1500000
espflash monitor --elf target/xtensa-esp32s3-none-elf/debug/light_wifi
```

#### esp32c3

```sh
# Wifi credentials should be valid only if you plan to run the `light_eth` "ethernet" example.
# The `light_wifi` example gets your Wifi settings from the Matter Controller automatically.
export WIFI_SSID=foo
export WIFI_PASS=bar

cd esp
cargo +nightly build --target riscv32imc-unknown-none-elf --no-default-features --features esp32c3,wifi

# Replace `light_wifi` with `light_eth` below to flash the "Ethernet" example
espflash flash target/riscv32imc-unknown-none-elf/debug/light_wifi --baud 1500000
espflash monitor --elf target/riscv32imc-unknown-none-elf/debug/light_wifi
```

#### esp32c6

```sh
# Wifi credentials should be valid only if you plan to run the `light_eth` "ethernet" example.
# The `light_wifi` example gets your Wifi settings from the Matter Controller automatically.
export WIFI_SSID=foo
export WIFI_PASS=bar

cd esp
cargo +nightly build --target riscv32imac-unknown-none-elf --no-default-features --features esp32c6,wifi

# Replace `light_wifi` with `light_eth` below to flash the "Ethernet" example
# Replace `light_wifi` with `light_thread` below to flash the Ethernet example (you'll need an esp32c6 or esp32h2)
espflash flash target/riscv32imac-unknown-none-elf/debug/light_wifi --baud 1500000
espflash monitor --elf target/riscv32imac-unknown-none-elf/debug/light_wifi
```

### Nordic nRF52840

```sh
# Thread dataset should be valid only if you plan to run the `light_eth` "ethernet" example.
# Get the Thread dataset from a commissioned device (`ot cli dataset active -x`) 
# or from your Thread Border Router and set the environment variable
export THREAD_DATASET="000003..."

cd nrf

# Replace `light_thread` with `light_eth` below to flash the Ethernet example
cargo run --bin light_thread
```

### Nordic nRF54L15

Only the `light_thread_coex` example runs here: `light_thread` and `light_eth` drive
the `embassy-nrf` IEEE 802.15.4 radio directly, and that one exists on the nRF52
family only. The chip is picked with a Cargo feature, and the target and the
`probe-rs` chip name have to be matched to it by hand.

Note that `--no-default-features` drops the default BLE host backend along with the
default chip, so the backend has to be named too - `trouble` here, or `nimble` for
the NimBLE host.

```sh
cd nrf

cargo run --no-default-features --features nrf54l15,trouble \
    --target thumbv8m.main-none-eabihf \
    --config 'target."cfg(all(target_arch = \"arm\", target_os = \"none\"))".runner = "probe-rs run --chip nRF54L15"' \
    --bin light_thread_coex
```
