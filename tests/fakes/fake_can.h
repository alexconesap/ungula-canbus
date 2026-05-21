// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <vector>

#include "ungula/hal/can/can_frame.h"
#include "ungula/hal/can/i_can.h"

namespace ungula::canbus::tests
{

/// In-memory `ICan` for unit tests.
///
/// - `send()` records the frame into `sent` and (optionally)
///   auto-queues a canned reply for `auto_reply`.
/// - `receive()` pops the next frame from `queued_rx`.
/// - `isBusOff()` reports whatever `bus_off` is set to.
///
/// Tests drive the fake by:
///   1. Pre-queueing replies the device-under-test should observe
///      (`queueRx(...)`).
///   2. Calling the protocol op under test.
///   3. Asserting on `sent` (what went out) and on the op's return.
class FakeCan : public ungula::hal::can::ICan {
    public:
        // ---- ICan -------------------------------------------------------
        bool send(const ungula::hal::can::CanFrame &frame, uint32_t /*timeoutMs*/) override
        {
                if (fail_next_send) {
                        fail_next_send = false;
                        return false;
                }
                sent.push_back(frame);
                // Per-TX-id mapping: useful for scanNodes-style tests
                // where the reply has to arrive immediately after the
                // specific probe (not pre-queued ahead of all probes).
                auto it = reply_for_tx_id.find(frame.id);
                if (it != reply_for_tx_id.end()) {
                        queued_rx.push_back(it->second);
                }
                if (auto_reply.has_value()) {
                        queued_rx.push_back(*auto_reply);
                        auto_reply.reset();
                }
                return true;
        }

        int32_t receive(ungula::hal::can::CanFrame &out, uint32_t /*timeoutMs*/) override
        {
                if (queued_rx.empty()) {
                        return 0;
                }
                out = queued_rx.front();
                queued_rx.pop_front();
                return 1;
        }

        bool isBusOff() const override
        {
                return bus_off;
        }

        bool recoverFromBusOff() override
        {
                bus_off = false;
                return true;
        }

        bool stop() override
        {
                return true;
        }

        // ---- Test knobs -------------------------------------------------
        /// Frames the device-under-test transmitted, in order.
        std::vector<ungula::hal::can::CanFrame> sent;
        /// Frames that will be returned by `receive()` in FIFO order.
        std::deque<ungula::hal::can::CanFrame> queued_rx;
        /// When set, the next `send()` queues this frame as a reply
        /// (single-shot - cleared after use). Use for ergonomic
        /// "send / immediately reply" tests.
        struct ReplySlot {
                ungula::hal::can::CanFrame frame;
                bool has_value_ = false;
                void reset()
                {
                        has_value_ = false;
                }
                bool has_value() const
                {
                        return has_value_;
                }
                ungula::hal::can::CanFrame &operator*()
                {
                        return frame;
                }
                ReplySlot &operator=(const ungula::hal::can::CanFrame &f)
                {
                        frame = f;
                        has_value_ = true;
                        return *this;
                }
        } auto_reply;
        /// Trip to true to make the next `send()` return false (one-shot).
        bool fail_next_send = false;
        /// Reported by `isBusOff()`.
        bool bus_off = false;
        /// Per-TX-id reply mapping. When `send(frame)` lands and
        /// `frame.id` matches a key here, the mapped reply is queued
        /// into `queued_rx` immediately so the next `receive()`
        /// surfaces it. Use this for scan-style tests where the
        /// reply has to land after the matching probe (not pre-
        /// queued ahead of every probe).
        std::map<uint32_t, ungula::hal::can::CanFrame> reply_for_tx_id;

        // ---- Helpers ----------------------------------------------------
        void queueRx(uint32_t canId, uint8_t cmd, const uint8_t (&payload_after_cmd)[7])
        {
                ungula::hal::can::CanFrame f{};
                f.id         = canId;
                f.extendedId = false;
                f.dlc        = 8;
                f.data[0]    = cmd;
                std::memcpy(&f.data[1], payload_after_cmd, 7);
                queued_rx.push_back(f);
        }

        void queueRxFrame(const ungula::hal::can::CanFrame &f)
        {
                queued_rx.push_back(f);
        }

        void reset()
        {
                sent.clear();
                queued_rx.clear();
                auto_reply.reset();
                fail_next_send = false;
                bus_off = false;
        }
};

} // namespace ungula::canbus::tests
