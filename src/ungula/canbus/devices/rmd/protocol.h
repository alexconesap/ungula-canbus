// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/hal/can/can_frame.h"

/// MyActuator RMD V3 wire protocol.
///
/// Every constant the lib uses to talk to an RMD V3 actuator lives in
/// this file. Adding a non-motor RMD device (status display, etc.)
/// would land in `devices/rmd/<thing>.h` alongside `motor.h` and
/// reuse these constants - the addressing scheme (`TX_BASE` /
/// `RX_BASE`) and broadcast id are common to every RMD device on
/// the bus.
///
/// V2 RMD firmware is NOT targeted; some command bytes overlap but
/// the payload layouts differ enough that the lib targets only V3
/// (production silicon since ~2020). The runtime probe
/// `CMD_READ_RUNTIME_MS` doubles as a V3 generation indicator -
/// V2 firmware does not respond.

namespace ungula::canbus::rmd
{

// ---- Frame addressing ----------------------------------------------

/// Outgoing single-motor command id: `TX_BASE + motor_id`. Valid
/// motor ids are 1..32.
constexpr uint32_t TX_BASE = 0x140;
/// Reply id from a single motor: `RX_BASE + motor_id`.
constexpr uint32_t RX_BASE = 0x240;
/// Broadcast id - currently used only by `setNodeId`. WARNING: every
/// motor currently at the targeted id will adopt the new id.
constexpr uint32_t TX_BROADCAST = 0x300;

/// Lowest valid motor id (RMD V3 protocol).
constexpr uint8_t MOTOR_ID_MIN = 1;
/// Highest valid motor id (RMD V3 protocol).
constexpr uint8_t MOTOR_ID_MAX = 32;

// ---- Command bytes (V3) --------------------------------------------

constexpr uint8_t CMD_READ_MODEL = 0x12; ///< read model + firmware version
constexpr uint8_t CMD_SHUTDOWN = 0x80; ///< motor off, free wheel
constexpr uint8_t CMD_STOP = 0x81; ///< active stop, holds position
constexpr uint8_t CMD_WAKE_UP = 0x88; ///< resume control after shutdown
constexpr uint8_t CMD_READ_STATUS_1 = 0x9A; ///< probe + temperature + voltage + error
constexpr uint8_t CMD_SET_NODE_ID = 0x79; ///< broadcast set-id
constexpr uint8_t CMD_SPEED_LOOP = 0xA2; ///< closed-loop speed
constexpr uint8_t CMD_POSITION_ABS = 0xA4; ///< multi-turn absolute + max speed
constexpr uint8_t CMD_POSITION_INCR = 0xA8; ///< incremental + max speed
constexpr uint8_t CMD_READ_POSITION = 0x60; ///< multi-turn angle, int32 LE 0.01 deg
constexpr uint8_t CMD_SET_ZERO_OFFSET = 0x64; ///< current position becomes zero (ROM write)
constexpr uint8_t CMD_READ_RUNTIME_MS = 0xB1; ///< V3 "read system runtime", ms since power-on

// ---- Pack / unpack helpers (little-endian, RMD wire format) -------
//
// Inlined so the compiler folds them into the frame builder math
// without any function-call overhead. All RMD multi-byte fields are
// LE on the wire regardless of host architecture.

inline void pack_le16(uint8_t *buf, uint16_t value)
{
        buf[0] = static_cast<uint8_t>(value & 0xFFU);
        buf[1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
}

inline uint16_t unpack_le16(const uint8_t *buf)
{
        return static_cast<uint16_t>(static_cast<uint16_t>(buf[0]) |
                                     (static_cast<uint16_t>(buf[1]) << 8));
}

inline void pack_le32(uint8_t *buf, int32_t value)
{
        const uint32_t u = static_cast<uint32_t>(value);
        buf[0] = static_cast<uint8_t>(u & 0xFFU);
        buf[1] = static_cast<uint8_t>((u >> 8) & 0xFFU);
        buf[2] = static_cast<uint8_t>((u >> 16) & 0xFFU);
        buf[3] = static_cast<uint8_t>((u >> 24) & 0xFFU);
}

inline int32_t unpack_le32(const uint8_t *buf)
{
        const uint32_t u = static_cast<uint32_t>(buf[0]) | (static_cast<uint32_t>(buf[1]) << 8) |
                           (static_cast<uint32_t>(buf[2]) << 16) |
                           (static_cast<uint32_t>(buf[3]) << 24);
        return static_cast<int32_t>(u);
}

// ---- Frame builders ------------------------------------------------

/// Build a zeroed 8-byte TX frame addressed to `motor_id` with `cmd`
/// in byte 0. Caller fills in any additional payload bytes.
inline ungula::hal::can::CanFrame makeTx(uint8_t motor_id, uint8_t cmd)
{
        ungula::hal::can::CanFrame tx{};
        tx.id = TX_BASE + motor_id;
        tx.extendedId = false;
        tx.dlc = 8;
        tx.data[0] = cmd;
        return tx;
}

} // namespace ungula::canbus::rmd
