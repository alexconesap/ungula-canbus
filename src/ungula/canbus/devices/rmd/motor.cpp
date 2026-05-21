// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include "ungula/canbus/devices/rmd/motor.h"

#include <cstddef>

#include "ungula/canbus/bus_helpers.h"

namespace ungula::canbus::rmd
{

namespace
{

        // Per-motor-id cache for sendSpeedIfChanged. Index 0 is unused so the
        // motor-id maps directly to the array index. Cache lives at file
        // scope; lib clients that need a per-bus cache (multiple independent
        // CAN segments in one process) would need to extend this, but in
        // practice every project to date has one bus.
        constexpr size_t kCacheSlots = MOTOR_ID_MAX + 1;
        int32_t s_last_speed_centidps[kCacheSlots] = {};
        bool s_last_speed_valid[kCacheSlots] = {};

        inline bool validId(uint8_t motor_id)
        {
                return motor_id >= MOTOR_ID_MIN && motor_id <= MOTOR_ID_MAX;
        }

} // namespace

// =====================================================================
// Speed loop
// =====================================================================

bool sendSpeed(ungula::hal::can::ICan &bus, uint8_t motor_id, int32_t speed_centidps)
{
        if (!validId(motor_id)) {
                return false;
        }
        auto tx = makeTx(motor_id, CMD_SPEED_LOOP);
        // bytes 1..3 are reserved (zero from makeTx).
        pack_le32(&tx.data[4], speed_centidps);
        return bus.send(tx, 20);
}

bool sendSpeedIfChanged(ungula::hal::can::ICan &bus, uint8_t motor_id, int32_t speed_centidps)
{
        if (!validId(motor_id)) {
                return false;
        }
        if (s_last_speed_valid[motor_id] && s_last_speed_centidps[motor_id] == speed_centidps) {
                return true; // no change, no CAN traffic
        }
        if (!sendSpeed(bus, motor_id, speed_centidps)) {
                // Send failed; leave the cache invalid so the next
                // call retries.
                return false;
        }
        s_last_speed_centidps[motor_id] = speed_centidps;
        s_last_speed_valid[motor_id] = true;
        return true;
}

void invalidateSpeedCache(uint8_t motor_id)
{
        if (motor_id == 0) {
                for (size_t i = 0; i < kCacheSlots; ++i) {
                        s_last_speed_valid[i] = false;
                }
                return;
        }
        if (motor_id <= MOTOR_ID_MAX) {
                s_last_speed_valid[motor_id] = false;
        }
}

// =====================================================================
// State control
// =====================================================================

bool stop(ungula::hal::can::ICan &bus, uint8_t motor_id)
{
        if (!validId(motor_id)) {
                return false;
        }
        invalidateSpeedCache(motor_id);
        return bus.send(makeTx(motor_id, CMD_STOP), 20);
}

bool shutdown(ungula::hal::can::ICan &bus, uint8_t motor_id)
{
        if (!validId(motor_id)) {
                return false;
        }
        invalidateSpeedCache(motor_id);
        return bus.send(makeTx(motor_id, CMD_SHUTDOWN), 20);
}

bool wakeUp(ungula::hal::can::ICan &bus, uint8_t motor_id)
{
        if (!validId(motor_id)) {
                return false;
        }
        invalidateSpeedCache(motor_id);
        return bus.send(makeTx(motor_id, CMD_WAKE_UP), 20);
}

// =====================================================================
// Position moves
// =====================================================================

bool moveAbsolute(ungula::hal::can::ICan &bus, uint8_t motor_id, int32_t target_centideg,
                  uint16_t max_speed_dps)
{
        if (!validId(motor_id)) {
                return false;
        }
        invalidateSpeedCache(motor_id);
        auto tx = makeTx(motor_id, CMD_POSITION_ABS);
        // byte 1 reserved.
        pack_le16(&tx.data[2], max_speed_dps);
        pack_le32(&tx.data[4], target_centideg);
        return bus.send(tx, 20);
}

bool moveRelative(ungula::hal::can::ICan &bus, uint8_t motor_id, int32_t delta_centideg,
                  uint16_t max_speed_dps)
{
        if (!validId(motor_id)) {
                return false;
        }
        invalidateSpeedCache(motor_id);
        auto tx = makeTx(motor_id, CMD_POSITION_INCR);
        pack_le16(&tx.data[2], max_speed_dps);
        pack_le32(&tx.data[4], delta_centideg);
        return bus.send(tx, 20);
}

// =====================================================================
// Reads
// =====================================================================

bool readPosition(ungula::hal::can::ICan &bus, uint8_t motor_id, int32_t *out_centideg)
{
        if (!validId(motor_id)) {
                return false;
        }
        const auto tx = makeTx(motor_id, CMD_READ_POSITION);
        ungula::hal::can::CanFrame rx{};
        if (!sendAndAwaitReply(bus, tx, RX_BASE + motor_id, CMD_READ_POSITION, rx, 50)) {
                return false;
        }
        if (out_centideg != nullptr) {
                *out_centideg = unpack_le32(&rx.data[4]);
        }
        return true;
}

bool readStatusRaw(ungula::hal::can::ICan &bus, uint8_t motor_id, uint8_t out_payload[8])
{
        if (!validId(motor_id)) {
                return false;
        }
        const auto tx = makeTx(motor_id, CMD_READ_STATUS_1);
        ungula::hal::can::CanFrame rx{};
        if (!sendAndAwaitReply(bus, tx, RX_BASE + motor_id, CMD_READ_STATUS_1, rx, 50)) {
                return false;
        }
        for (int i = 0; i < 8; ++i) {
                out_payload[i] = rx.data[i];
        }
        return true;
}

bool readModel(ungula::hal::can::ICan &bus, uint8_t motor_id, uint8_t out_payload[8])
{
        if (!validId(motor_id)) {
                return false;
        }
        const auto tx = makeTx(motor_id, CMD_READ_MODEL);
        ungula::hal::can::CanFrame rx{};
        if (!sendAndAwaitReply(bus, tx, RX_BASE + motor_id, CMD_READ_MODEL, rx, 100)) {
                return false;
        }
        for (int i = 0; i < 8; ++i) {
                out_payload[i] = rx.data[i];
        }
        return true;
}

bool readSystemRuntime(ungula::hal::can::ICan &bus, uint8_t motor_id, uint8_t out_payload[8])
{
        if (!validId(motor_id)) {
                return false;
        }
        const auto tx = makeTx(motor_id, CMD_READ_RUNTIME_MS);
        ungula::hal::can::CanFrame rx{};
        if (!sendAndAwaitReply(bus, tx, RX_BASE + motor_id, CMD_READ_RUNTIME_MS, rx, 50)) {
                return false;
        }
        for (int i = 0; i < 8; ++i) {
                out_payload[i] = rx.data[i];
        }
        return true;
}

// =====================================================================
// Zero set
// =====================================================================

bool setCurrentPositionAsZero(ungula::hal::can::ICan &bus, uint8_t motor_id)
{
        if (!validId(motor_id)) {
                return false;
        }
        invalidateSpeedCache(motor_id);
        return bus.send(makeTx(motor_id, CMD_SET_ZERO_OFFSET), 20);
}

// =====================================================================
// Discovery + addressing
// =====================================================================

bool setNodeId(ungula::hal::can::ICan &bus, uint8_t new_node_id)
{
        if (!validId(new_node_id)) {
                return false;
        }
        ungula::hal::can::CanFrame cmd{};
        cmd.id = TX_BROADCAST;
        cmd.extendedId = false;
        cmd.dlc = 8;
        cmd.data[0] = CMD_SET_NODE_ID;
        cmd.data[2] = 0x00; // 0 = write, 1 = read
        cmd.data[7] = new_node_id; // RMD V3: new id lives in byte 7
        return bus.send(cmd, 20);
}

int scanNodes(ungula::hal::can::ICan &bus)
{
        drainRx(bus);
        return ungula::canbus::scanNodes(
            bus, MOTOR_ID_MIN, MOTOR_ID_MAX,
            // probe: send a status read to TX_BASE + id
            [](uint8_t id) { return makeTx(id, CMD_READ_STATUS_1); },
            // match: reply on RX_BASE + id with command echo
            [](const ungula::hal::can::CanFrame &rx, uint8_t expectedId) {
                    return rx.id == (RX_BASE + expectedId) && rx.data[0] == CMD_READ_STATUS_1;
            },
            /*replyTimeoutMs=*/10);
}

} // namespace ungula::canbus::rmd
