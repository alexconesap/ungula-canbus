// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "ungula/hal/can/can_frame.h"
#include "ungula/hal/can/i_can.h"

namespace ungula::canbus
{

/// Drain every pending frame from the RX queue. Useful before a
/// discovery sweep so prior frames don't pollute the scan.
void drainRx(ungula::hal::can::ICan &bus);

/// Send `tx`, then poll the bus for up to `timeoutMs` waiting for the
/// next received frame to match `expectedReplyId` AND `data[0] ==
/// expectedCommandByte`. Copies the matching frame into `out` and
/// returns true. Returns false if:
///   - the TX failed (queue full, bus-off, ...),
///   - no frame arrived within the timeout,
///   - the first frame to arrive had the wrong canId or command byte.
///
/// The "first-frame-only" semantic matches what every per-brand
/// device protocol needs: hosts that want to multiplex multiple
/// in-flight requests should manage their own RX loop instead.
bool sendAndAwaitReply(ungula::hal::can::ICan &bus, const ungula::hal::can::CanFrame &tx,
                       uint32_t expectedReplyId, uint8_t expectedCommandByte,
                       ungula::hal::can::CanFrame &out, uint32_t timeoutMs = 50);

/// Probe every node id in `[firstId, lastId]` for a reply, counting
/// matches. Per-id flow:
///
///   1. Build a probe frame via `probe(id)`.
///   2. `bus.send(probe)`. Skip the id on TX failure.
///   3. Wait up to `replyTimeoutMs` for a frame, check `matches(rx, id)`.
///   4. If the predicate accepts the frame, increment the count.
///
/// `probe` is anything callable as `CanFrame(uint8_t id)`.
/// `matches` is anything callable as `bool(const CanFrame&, uint8_t expectedId)`.
/// Both are deduced — lambdas, function pointers, custom functors all
/// work; nothing is heap-allocated.
///
/// Caller is responsible for draining the RX queue first if leftover
/// frames could pollute the count - use `drainRx(bus)`.
template <typename ProbeFn, typename MatchFn>
int scanNodes(ungula::hal::can::ICan &bus, uint8_t firstId, uint8_t lastId, ProbeFn probe,
              MatchFn matches, uint32_t replyTimeoutMs = 10)
{
        if (firstId == 0 || firstId > lastId) {
                return 0;
        }
        int found = 0;
        for (uint16_t i = firstId; i <= lastId; ++i) {
                const uint8_t id = static_cast<uint8_t>(i);
                ungula::hal::can::CanFrame tx = probe(id);
                if (!bus.send(tx, 20)) {
                        continue;
                }
                ungula::hal::can::CanFrame rx{};
                const int32_t got = bus.receive(rx, replyTimeoutMs);
                if (got == 1 && matches(rx, id)) {
                        ++found;
                }
        }
        return found;
}

} // namespace ungula::canbus
