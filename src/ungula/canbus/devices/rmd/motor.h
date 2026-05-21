// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/canbus/devices/rmd/protocol.h"
#include "ungula/hal/can/i_can.h"

/// MyActuator RMD V3 motor operations.
///
/// Free functions that take an `ICan &` and a motor id. Every
/// function builds the proper TX frame, sends it, and (where the
/// operation expects a reply) waits up to the documented timeout
/// for a matching response. Errors come back as `bool false` - the
/// host logs whatever it wants on failure.
///
/// `motor_id` is 1..32 (`MOTOR_ID_MIN`..`MOTOR_ID_MAX`). Out-of-range
/// ids return false without touching the bus.

namespace ungula::canbus::rmd
{

// =====================================================================
// Speed loop (CMD_SPEED_LOOP = 0xA2)
// =====================================================================

/// Send a closed-loop speed command. `speed_centidps` is in 0.01
/// deg/s units (RMD convention); sign sets direction. Returns true
/// if the frame was queued and ACKed.
bool sendSpeed(ungula::hal::can::ICan &bus, uint8_t motor_id, int32_t speed_centidps);

/// Same as `sendSpeed` but skips the CAN write when `speed_centidps`
/// is bit-identical to the last value sent to this motor via this
/// function. Avoids re-asserting unchanged setpoints, which on some
/// V3 firmware re-triggers the internal acceleration ramp and
/// produces audible / visible stutter at the loop rate.
///
/// Cache is per-motor-id (valid range 1..32) and lives at file
/// scope. Returns true if either no send was needed (value
/// unchanged) or the send succeeded. On send failure the cache is
/// NOT updated, so the next call retries.
bool sendSpeedIfChanged(ungula::hal::can::ICan &bus, uint8_t motor_id, int32_t speed_centidps);

/// Drop the cached last-speed for `motor_id`, forcing the next
/// `sendSpeedIfChanged` to actually transmit. Call after any
/// operation that may have changed the motor's speed state behind
/// the cache's back (shutdown, stop, moveAbsolute, moveRelative,
/// motor reboot, bus-off recovery). Passing 0 invalidates every
/// entry.
void invalidateSpeedCache(uint8_t motor_id = 0);

// =====================================================================
// State control
// =====================================================================

/// Active stop. Motor holds its current position via the internal
/// loop - still draws current. Use this to pause motion safely.
bool stop(ungula::hal::can::ICan &bus, uint8_t motor_id);

/// Disable the motor entirely. No torque, free to spin by hand.
/// Use `wakeUp` to resume control.
bool shutdown(ungula::hal::can::ICan &bus, uint8_t motor_id);

/// Resume the internal control loop after `shutdown`. Position
/// commands resume working.
bool wakeUp(ungula::hal::can::ICan &bus, uint8_t motor_id);

// =====================================================================
// Position moves (multi-turn)
// =====================================================================

/// Move to absolute target. `target_centideg` is in 0.01 deg
/// (signed, multi-turn - the motor tracks revolutions, not just
/// 0..360). `max_speed_dps` caps the speed during the move in
/// deg/s (NOT centideg/s - that's the V3 protocol's quirk for the
/// max-speed field).
bool moveAbsolute(ungula::hal::can::ICan &bus, uint8_t motor_id, int32_t target_centideg,
                  uint16_t max_speed_dps);

/// Move by a signed delta relative to current position.
/// `delta_centideg` is in 0.01 deg; sign sets direction.
/// `max_speed_dps` same as `moveAbsolute` (deg/s, not centi).
bool moveRelative(ungula::hal::can::ICan &bus, uint8_t motor_id, int32_t delta_centideg,
                  uint16_t max_speed_dps);

// =====================================================================
// Reads
// =====================================================================

/// Read the current multi-turn encoder position. Writes the result
/// (int32 in 0.01 deg) into `*out_centideg`. Returns true on a
/// valid reply. Cumulative across revolutions; survives until the
/// motor is power-cycled (internal counter, not derived from the
/// magnetic encoder's single-turn value).
bool readPosition(ungula::hal::can::ICan &bus, uint8_t motor_id, int32_t *out_centideg);

/// Read motor status 1 (0x9A): temperature, voltage, error bits,
/// etc. Writes the raw 8-byte payload into `out_payload`. The exact
/// byte layout varies between V3 sub-builds; decode per the motor's
/// manual. Common layout:
///   byte 1   : temperature (int8, deg C)
///   bytes 3-4: bus voltage (uint16 LE, 0.1 V units)
///   bytes 6-7: error code  (uint16 LE bitfield)
bool readStatusRaw(ungula::hal::can::ICan &bus, uint8_t motor_id, uint8_t out_payload[8]);

/// Read model + firmware version (CMD_READ_MODEL = 0x12). Writes the
/// 8-byte response into `out_payload`. RMD V3 layout:
///   byte 0   : echo of 0x12
///   byte 1   : firmware major
///   byte 2   : firmware minor
///   bytes 3-7: model + build info (varies by generation)
/// Cheap, side-effect-free probe - good for an existence check at
/// bring-up.
bool readModel(ungula::hal::can::ICan &bus, uint8_t motor_id, uint8_t out_payload[8]);

/// Read the motor's runtime counter (CMD_READ_RUNTIME_MS = 0xB1).
/// Writes the 8-byte response into `out_payload`. Bytes 4..7 are
/// uptime in milliseconds (uint32 LE) since the motor was powered
/// on. Recognition of this command is a strong indicator that the
/// motor speaks V3 protocol - V2 firmware doesn't respond.
bool readSystemRuntime(ungula::hal::can::ICan &bus, uint8_t motor_id, uint8_t out_payload[8]);

// =====================================================================
// Zero set
// =====================================================================

/// Write the current encoder position as the new multi-turn zero
/// offset. The motor reboots after accepting this command.
///
/// CAUTION: this persists to ROM. RMD documents a limited number of
/// flash erase cycles (manufacturer cites the order of 10k). Do
/// NOT call this in normal operation; it's a bring-up /
/// calibration step.
bool setCurrentPositionAsZero(ungula::hal::can::ICan &bus, uint8_t motor_id);

// =====================================================================
// Discovery + addressing
// =====================================================================

/// Broadcast a set-node-id command (0x79) on `TX_BROADCAST`. The
/// CALLER MUST ENSURE only one motor is on the bus, or every motor
/// currently at the targeted id will adopt the new one. Motor
/// reboots after accepting.
bool setNodeId(ungula::hal::can::ICan &bus, uint8_t new_node_id);

/// Probe ids 1..32 with `CMD_READ_STATUS_1` and count how many
/// reply on `RX_BASE + id` with the matching command byte. Returns
/// the count. Drains the RX queue first so prior frames don't
/// pollute the scan. Boot-time wiring / bitrate sanity check.
int scanNodes(ungula::hal::can::ICan &bus);

} // namespace ungula::canbus::rmd
