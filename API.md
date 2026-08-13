# `lib_canbus` API reference

Public surface of UngulaCanbus 0.1.1. For an overview + copy-pastable
examples, read `README.md` first; this file is the reference you reach
for when you already know what you're building.

Every declaration shown here lives in `namespace ungula::canbus` unless
explicitly nested further. Per-brand protocols live in
`ungula::canbus::<brand>` (today only `ungula::canbus::rmd`).

## API stability (0.x)

The version is `0.x` on purpose. The shape of the API — free functions
over an `ICan &`, per-brand namespaces, protocol constants in
`<brand>/protocol.h` — is settled and unlikely to move. These parts are
not:

- **Error reporting.** Every operation returns a bare `bool`. There is no
  way to tell "no reply" from "driver error" from "wrong frame". A status
  enum in the shape of the other Ungula libraries is the obvious next
  step, and it changes every signature that currently returns `bool`.
- **Timeouts.** The 20 ms TX timeout and the 50 / 100 / 10 ms reply
  timeouts are hardcoded at each call site and cannot be overridden.
  Expect them to become parameters or a config struct.
- **The speed cache.** File-scope, single-bus, invalidated by hand. If it
  becomes per-bus state it turns into an object and the free-function
  signatures change with it.
- **Frame validation.** `dlc`, `extendedId` and `remote` are not checked
  on received frames. Tightening that will change which frames are
  accepted, i.e. observable behaviour.

Pin the version if you depend on any of the above.

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

Protocol-agnostic helpers in `namespace ungula::canbus`. Every one of
them is **blocking, task-context only** — never call them from an ISR.

```cpp
void drainRx(ungula::hal::can::ICan &bus);

bool sendAndAwaitReply(ungula::hal::can::ICan &bus,
                       const ungula::hal::can::CanFrame &tx,
                       uint32_t expectedReplyId,
                       uint8_t  expectedCommandByte,
                       ungula::hal::can::CanFrame &out,
                       uint32_t timeoutMs = 50);

template <typename ProbeFn, typename MatchFn>
int scanNodes(ungula::hal::can::ICan &bus,
              uint8_t firstId, uint8_t lastId,
              ProbeFn probe, MatchFn matches,
              uint32_t replyTimeoutMs = 10);
```

### `drainRx(bus)`

Pops frames with a zero timeout until `receive()` stops returning 1.
Call it before a discovery sweep so leftover frames don't pollute the
count.

Caveat: the loop has **no iteration cap**. On a bus where traffic
arrives at least as fast as it is drained (a babbling node, a motor
streaming status), it does not terminate and the calling task hangs
until the watchdog fires. Only call it when the bus is known to be
quiet — typically at bring-up, before any node has been commanded.

### `sendAndAwaitReply(...)`

Sends `tx`, then waits up to `timeoutMs` for **exactly one** frame and
accepts it only if `out.id == expectedReplyId` and
`out.data[0] == expectedCommandByte`. Returns `true` and leaves the
frame in `out`. Returns `false` when the TX failed, nothing arrived, or
the one frame that arrived did not match.

Matching semantics you must design around:

- **First frame only.** It does not keep reading until the timeout
  expires; any unrelated frame that happens to arrive first makes the
  call fail. On a bus with more than one talker, reads fail
  intermittently — retry at the call site.
- **No queue hygiene between calls.** A reply that arrives after its
  request timed out stays queued and is consumed by the *next*
  `sendAndAwaitReply()`. If both requests targeted the same id and
  command, the stale frame is accepted as fresh and the caller silently
  gets one-cycle-old data. Call `drainRx()` between unrelated
  transactions if this matters.
- **`dlc` is not checked** before reading `data[0]`, and
  **`extendedId` is not checked** against the expected id width. A
  zero-length frame or a 29-bit frame whose id collides with
  `expectedReplyId` can be accepted. Validate `out.dlc` and
  `out.extendedId` yourself if the bus is mixed.
- **A driver error is indistinguishable from a timeout.** `receive()`
  returning `-1` is treated exactly like `0`. Poll `bus.isBusOff()`
  separately — see "Bus-off" below.

### `scanNodes(bus, firstId, lastId, probe, matches, replyTimeoutMs)`

Probes each id in `[firstId, lastId]`, counting the ones that answer.
Per id: build `probe(id)`, `send()` (skip the id on TX failure), wait
up to `replyTimeoutMs` for one frame, count it if `matches(rx, id)`.
Returns 0 immediately when `firstId == 0` or `firstId > lastId`.

`probe` is anything callable as `CanFrame(uint8_t)`; `matches` is
anything callable as `bool(const CanFrame&, uint8_t)`. Both are
deduced, so lambdas and function pointers work and nothing is
heap-allocated. The same first-frame-only and `dlc` / `extendedId`
caveats as `sendAndAwaitReply` apply inside `matches` — the frame you
receive is unvalidated.

Does **not** drain the RX queue for you; call `drainRx(bus)` first.

### Bus-off

`ICan::isBusOff()` and `ICan::recoverFromBusOff()` exist on the
interface but **nothing in UngulaCanbus calls them**. After the
controller drops to bus-off, every helper here keeps returning `false`
with no distinct error and no recovery attempt. Host code owns the
bus-off watch:

```cpp
if (bus.isBusOff()) {
    bus.recoverFromBusOff();
    ungula::canbus::rmd::invalidateSpeedCache();  // drop stale setpoints
}
```

## Devices

### RMD V3 (MyActuator)

`namespace ungula::canbus::rmd`. Two headers:
`devices/rmd/protocol.h` (wire constants + pack/unpack + frame
builder) and `devices/rmd/motor.h` (the operations).

Only RMD **V3** is targeted. Some V2 command bytes overlap but the
payload layouts differ; `CMD_READ_RUNTIME_MS` doubles as a V3
generation probe because V2 firmware does not answer it.

#### Wire constants (`protocol.h`)

| Constant | Value | Meaning |
| --- | --- | --- |
| `TX_BASE` | `0x140` | Command id for one motor is `TX_BASE + motor_id` |
| `RX_BASE` | `0x240` | Reply id from one motor is `RX_BASE + motor_id` |
| `TX_BROADCAST` | `0x300` | Broadcast id; used only by `setNodeId()` |
| `MOTOR_ID_MIN` / `MOTOR_ID_MAX` | `1` / `32` | Valid motor id range |

Command bytes: `CMD_READ_MODEL` `0x12`, `CMD_READ_POSITION` `0x60`,
`CMD_SET_ZERO_OFFSET` `0x64`, `CMD_SET_NODE_ID` `0x79`,
`CMD_SHUTDOWN` `0x80`, `CMD_STOP` `0x81`, `CMD_WAKE_UP` `0x88`,
`CMD_READ_STATUS_1` `0x9A`, `CMD_SPEED_LOOP` `0xA2`,
`CMD_POSITION_ABS` `0xA4`, `CMD_POSITION_INCR` `0xA8`,
`CMD_READ_RUNTIME_MS` `0xB1`.

#### Helpers (`protocol.h`)

```cpp
void     pack_le16(uint8_t *buf, uint16_t value);
uint16_t unpack_le16(const uint8_t *buf);
void     pack_le32(uint8_t *buf, int32_t value);
int32_t  unpack_le32(const uint8_t *buf);

ungula::hal::can::CanFrame makeTx(uint8_t motor_id, uint8_t cmd);
```

All RMD multi-byte fields are little-endian on the wire regardless of
host architecture; use these rather than `memcpy` of a struct. They do
no bounds checking — the caller guarantees 2 / 4 readable bytes.
`unpack_le16` / `unpack_le32` are there for decoding the raw payloads
returned by `readStatusRaw()` and friends.

`makeTx()` returns a zeroed 8-byte standard frame with
`id = TX_BASE + motor_id` and `data[0] = cmd`; the caller fills the
rest. It does **not** validate `motor_id` — passing something above 32
silently produces a colliding id. Use the `motor.h` operations, which
do validate, unless you are building a frame the library has no
function for.

#### Operations (`motor.h`)

Free functions, all taking `ICan &bus` first. Every one validates
`motor_id` against 1..32 and returns `false` without touching the bus
when it is out of range. Failures are a plain `false` — no error code,
no logging (the host logs what it wants).

Fire-and-forget writes. These queue a frame with a 20 ms TX timeout and
return whether it was queued; they do **not** wait for the motor to
confirm anything:

| Function | Frame | Notes |
| --- | --- | --- |
| `bool sendSpeed(bus, motor_id, int32_t speed_centidps)` | `0xA2` | Speed in 0.01 deg/s, signed. Payload at bytes 4..7 |
| `bool stop(bus, motor_id)` | `0x81` | Active stop, holds position, still draws current |
| `bool shutdown(bus, motor_id)` | `0x80` | Motor off, free to spin by hand |
| `bool wakeUp(bus, motor_id)` | `0x88` | Resume control after `shutdown` |
| `bool moveAbsolute(bus, motor_id, int32_t target_centideg, uint16_t max_speed_dps)` | `0xA4` | Multi-turn absolute. Target in 0.01 deg; **`max_speed_dps` is deg/s, not centi** |
| `bool moveRelative(bus, motor_id, int32_t delta_centideg, uint16_t max_speed_dps)` | `0xA8` | Signed delta, same units as above |
| `bool setCurrentPositionAsZero(bus, motor_id)` | `0x64` | **Writes ROM.** Motor reboots. Bring-up only — RMD cites ~10k erase cycles |
| `bool setNodeId(bus, new_node_id)` | `0x79` on `TX_BROADCAST` | Broadcast. Every motor at the targeted id adopts the new one — put exactly one motor on the bus. Motor reboots |

Reads. These use `sendAndAwaitReply()` and inherit all of its matching
caveats above:

| Function | Cmd / timeout | Notes |
| --- | --- | --- |
| `bool readPosition(bus, motor_id, int32_t *out_centideg)` | `0x60`, 50 ms | Multi-turn encoder position in 0.01 deg, decoded from bytes 4..7. Cumulative across revolutions; resets on motor power-cycle. `out_centideg == nullptr` is tolerated (reply is still consumed) |
| `bool readStatusRaw(bus, motor_id, uint8_t out[8])` | `0x9A`, 50 ms | Raw payload. Typical layout: byte 1 temperature (int8 °C), bytes 3-4 bus voltage (uint16 LE, 0.1 V), bytes 6-7 error bitfield. Varies by V3 sub-build |
| `bool readModel(bus, motor_id, uint8_t out[8])` | `0x12`, 100 ms | byte 0 echoes `0x12`, byte 1 firmware major, byte 2 minor, bytes 3-7 model/build |
| `bool readSystemRuntime(bus, motor_id, uint8_t out[8])` | `0xB1`, 50 ms | Bytes 4..7 = uptime ms (uint32 LE). A reply also confirms V3 firmware |
| `int scanNodes(bus)` | `0x9A`, 10 ms/id | Drains RX, probes ids 1..32, returns the number that answered on `RX_BASE + id`. Boot-time wiring / bitrate check |

The three `out[8]` reads copy a full 8 bytes out of the reply frame
**regardless of its `dlc`**. Bytes past the reported length are
whatever the driver left in the frame buffer. They also do not check
`out` for null — unlike `readPosition`, passing `nullptr` is a crash.

#### The speed cache

```cpp
bool sendSpeedIfChanged(bus, motor_id, int32_t speed_centidps);
void invalidateSpeedCache(uint8_t motor_id = 0);   // 0 = all ids
```

`sendSpeedIfChanged()` skips the CAN write when the setpoint is
bit-identical to the last value this function sent to that id. Some V3
firmware re-triggers its internal acceleration ramp on every `0xA2`
frame, which shows up as audible stutter at the loop rate; this avoids
it. Returns `true` both when the send succeeded and when no send was
needed. On send failure the cache is left invalid so the next call
retries.

Things this cache implies:

- It is a **file-scope array indexed by motor id**, not per-bus state.
  Two `ICan` segments in the same firmware share one cache and will
  suppress each other's setpoints.
- It is not task-safe. Drive one motor id from one task.
- The library invalidates it inside `stop`, `shutdown`, `wakeUp`,
  `moveAbsolute`, `moveRelative` and `setCurrentPositionAsZero`.
  It does **not** invalidate on `setNodeId()`, so after re-addressing a
  motor the old id's entry still claims to be valid — call
  `invalidateSpeedCache()` yourself. Same after a motor power-cycle or a
  bus-off recovery, which the library cannot observe.

## Future hardware (placeholders, not implemented yet)

- `ungula::hal::can::Mcp2515Can` — classic CAN 2.0 via SPI MCP2515.
  Same `ICan` interface; drop-in for `Can`.
- `ungula::hal::can::ICanFd` + `ungula::hal::can::CanFd` — CAN-FD
  controller. Separate interface (different frame shape).

Until those land, UngulaCanbus is classic-CAN-only and the only
concrete bus is `lib_hal::can::Can`.
