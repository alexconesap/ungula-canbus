# UngulaCanbus (`lib_canbus`) 0.1.0

CAN-bus service layer + per-brand wire protocols for embedded C++17.
Sits on top of `lib_hal`'s classic-CAN abstraction (`ICan` /
`Can`) and underneath any application code that talks to a CAN
device. Hosts call brand-named operations (`ungula::canbus::rmd::sendSpeed`,
`...::scanNodes`, ...) without knowing CAN frame layouts or platform
specifics.

## What's in here

Generic bus services (protocol-agnostic):

- Templated node discovery (`bus_helpers::scanNodes`).
- RX queue drain.
- Send-and-await-matching-reply pattern.

Per-brand device protocols under `devices/<brand>/`:

- **MyActuator RMD V3** — motor speed / position / status, set-node-id
  broadcast, runtime probe.

Per-brand folders own the wire constants (TX_BASE, command bytes,
payload offsets) and the operations that use them. Adding a non-motor
device from the same brand: a second file in the same folder. Adding a
new brand: a new folder. **No library split per brand.**

## Two abstraction axes

UngulaCanbus relies on `lib_hal` to hide:

1. **MCU platform** — ESP32 today; STM32 etc. later. UngulaCanbus never
   touches ESP-IDF or vendor SDKs directly.
2. **CAN family** — classic CAN 2.0 today (TWAI on ESP32). Future
   external controllers (MCP2515) plug into the same `ICan` interface
   in `lib_hal`. CAN-FD lands as a parallel hierarchy (`ICanFd` /
   `CanFd`) on its own interface — not part of `ICan`.

Hosts that want a different CAN controller swap the `lib_hal` class
they construct; the UngulaCanbus protocols accept `ICan&` and don't
care which controller is on the other end.

## Hello world

```cpp
#include <ungula/hal/can/can.h>                      // ESP32 TWAI today
#include <ungula/canbus/devices/rmd/motor.h>         // RMD V3 protocol

ungula::hal::can::Can bus(0);
bus.begin(/*tx=*/21, /*rx=*/22, ungula::hal::can::BITRATE_1M);

// Discover responding nodes (probes 1..32).
const int found = ungula::canbus::rmd::scanNodes(bus);

// Hold position at 30 RPM forward (3000 centideg/s).
ungula::canbus::rmd::sendSpeed(bus, /*motor_id=*/1, 3000);
```

Host code passes `bus` directly — `Can` is an `ICan` by inheritance,
no adapter object needed.

## Testing

```shell
cd lib_canbus/tests
./1_build.sh
./2_run.sh
```

Host tests use a `FakeCan` (in-memory `ICan` impl) — no hardware
required for the bus_helpers / protocol / device tests. The actual
ESP32 TWAI driver is covered by `lib_hal/tests` and on-target loopback.

## Dependencies

- [UngulaHal](https://github.com/alexconesap/ungula-hal) defines the \`ICan\` interface + concrete CAN controller classes. Required.

## Acknowledgements

Thanks to Claude and ChatGPT for helping on generating this documentation.

## License

MIT License — see [LICENSE](license.txt) file.

---

## Arduino CLI symlink note (rarely relevant)

This library ships a flat forwarder header at `src/ungula_canbus.h` that
just `#include`s `ungula/canbus.h`. `library.properties` `includes=` points
at the forwarder.

It only exists to work around an Arduino CLI quirk: when the library is
consumed through a symlink, the CLI sometimes fails to discover headers
nested under `src/ungula/`. The flat forwarder fixes that scan.

**Host code keeps including the real header**:

```cpp
#include <ungula/canbus.h>
```

PlatformIO, ESP-IDF component builds, and plain CMake setups can ignore
the forwarder.
