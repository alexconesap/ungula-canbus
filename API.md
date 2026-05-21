# `lib_canbus` API reference

Public surface of UngulaCanbus 0.1.0. For an overview + copy-pastable
examples, read `README.md` first; this file is the reference you reach
for when you already know what you're building.

Every declaration shown here lives in `namespace ungula::canbus` unless
explicitly nested further. Per-brand protocols live in
`ungula::canbus::<brand>` (today only `ungula::canbus::rmd`).

## Headers you include

```cpp
// Always (or pick the umbrella):
#include <ungula/canbus.h>
// equivalent to including the pieces below directly:
//   #include <ungula/hal/can/can.h>            // Can class (concrete TWAI today)
//   #include <ungula/hal/can/i_can.h>          // ICan interface
//   #include <ungula/canbus/bus_helpers.h>     // scanNodes<>, drainRx, ...
//   #include <ungula/canbus/devices/rmd/motor.h>

// Per-brand: pull only what you actually use. Each device-class
// header transitively pulls its brand's protocol header.
#include <ungula/canbus/devices/rmd/motor.h>
```

## Bus interface (re-exported from lib_hal)

```cpp
namespace ungula::hal::can {

struct CanFrame {
    uint32_t id;
    bool     extendedId;
    bool     remote;
    uint8_t  dlc;
    uint8_t  data[8];
};

class ICan {
public:
    virtual ~ICan() = default;
    virtual bool    send(const CanFrame &, uint32_t timeoutMs = 50) = 0;
    virtual int32_t receive(CanFrame &out, uint32_t timeoutMs = 0)  = 0;
    virtual bool    isBusOff() const                                = 0;
    virtual bool    recoverFromBusOff()                             = 0;
    virtual bool    stop()                                          = 0;
};

class Can : public ICan { /* ESP32 TWAI on ESP_PLATFORM; stub elsewhere */ };

} // namespace ungula::hal::can
```

All UngulaCanbus operations take `ungula::hal::can::ICan &` so they
work against any `ICan` implementation. Host code typically
constructs a concrete `Can` (or future `Mcp2515Can`), calls
controller-specific `begin(...)`, then hands it to the protocol
operations as-is.

## Bus services (`bus_helpers.h`)

> Filled in step 3 of the lib_canbus rollout (templated `scanNodes`,
> `drainRx`, `sendAndAwaitReply`). This section will list signatures +
> usage when the file lands.

## Devices

### RMD V3 (MyActuator)

> Filled in step 3. Will document the wire constants in
> `devices/rmd/protocol.h` and the operations in
> `devices/rmd/motor.h`:
> `sendSpeed`, `sendSpeedIfChanged`, `invalidateSpeedCache`,
> `setNodeId`, `scanNodes`, `readSystemRuntime`, `stop`, `shutdown`,
> `wakeUp`, `moveAbsolute`, `moveRelative`, `readPosition`,
> `readStatusRaw`, `setCurrentPositionAsZero`.

## Future hardware (placeholders, not implemented yet)

- `ungula::hal::can::Mcp2515Can` — classic CAN 2.0 via SPI MCP2515.
  Same `ICan` interface; drop-in for `Can`.
- `ungula::hal::can::ICanFd` + `ungula::hal::can::CanFd` — CAN-FD
  controller. Separate interface (different frame shape).

Until those land, UngulaCanbus is classic-CAN-only and the only
concrete bus is `lib_hal::can::Can`.
