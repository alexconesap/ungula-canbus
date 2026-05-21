// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <cstdint>

#include "ungula/hal/can/can_frame.h"
#include "ungula/hal/can/i_can.h"

#include "ungula/canbus/bus_helpers.h"

namespace ungula::canbus
{

void drainRx(ungula::hal::can::ICan &bus)
{
        ungula::hal::can::CanFrame discard{};
        while (bus.receive(discard, 0) == 1) {
                // discard
        }
}

bool sendAndAwaitReply(ungula::hal::can::ICan &bus, const ungula::hal::can::CanFrame &tx,
                       uint32_t expectedReplyId, uint8_t expectedCommandByte,
                       ungula::hal::can::CanFrame &out, uint32_t timeoutMs)
{
        if (!bus.send(tx, 20)) {
                return false;
        }
        if (bus.receive(out, timeoutMs) != 1) {
                return false;
        }
        if (out.id != expectedReplyId) {
                return false;
        }
        if (out.data[0] != expectedCommandByte) {
                return false;
        }
        return true;
}

} // namespace ungula::canbus
