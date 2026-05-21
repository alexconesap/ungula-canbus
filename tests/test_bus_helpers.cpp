// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include "fakes/fake_can.h"
#include "ungula/canbus/bus_helpers.h"

using ungula::canbus::tests::FakeCan;
using ungula::hal::can::CanFrame;

namespace
{

CanFrame makeFrame(uint32_t id, uint8_t cmd, uint8_t b1 = 0)
{
        CanFrame f{};
        f.id      = id;
        f.dlc     = 8;
        f.data[0] = cmd;
        f.data[1] = b1;
        return f;
}

} // namespace

// ---- drainRx --------------------------------------------------------

TEST(BusHelpers, DrainRxEmptiesQueue)
{
        FakeCan bus;
        bus.queueRxFrame(makeFrame(0x100, 0xAA));
        bus.queueRxFrame(makeFrame(0x101, 0xBB));
        bus.queueRxFrame(makeFrame(0x102, 0xCC));

        ungula::canbus::drainRx(bus);

        CanFrame check{};
        EXPECT_EQ(bus.receive(check, 0), 0);
}

TEST(BusHelpers, DrainRxOnEmptyQueueIsNoOp)
{
        FakeCan bus;
        ungula::canbus::drainRx(bus); // must not crash / loop
}

// ---- sendAndAwaitReply ---------------------------------------------

TEST(BusHelpers, SendAndAwaitReplyMatchesCanIdAndCommand)
{
        FakeCan bus;
        bus.queueRxFrame(makeFrame(0x241, 0xA2, 0x42));

        CanFrame tx = makeFrame(0x141, 0xA2);
        CanFrame rx{};
        ASSERT_TRUE(ungula::canbus::sendAndAwaitReply(
            bus, tx, /*expectedReplyId=*/0x241, /*expectedCmd=*/0xA2, rx));
        EXPECT_EQ(rx.data[1], 0x42u);

        ASSERT_EQ(bus.sent.size(), 1u);
        EXPECT_EQ(bus.sent[0].id, 0x141u);
}

TEST(BusHelpers, SendAndAwaitReplyRejectsWrongCanId)
{
        FakeCan bus;
        bus.queueRxFrame(makeFrame(0x999, 0xA2)); // wrong reply id

        CanFrame tx = makeFrame(0x141, 0xA2);
        CanFrame rx{};
        EXPECT_FALSE(ungula::canbus::sendAndAwaitReply(
            bus, tx, /*expectedReplyId=*/0x241, /*expectedCmd=*/0xA2, rx));
}

TEST(BusHelpers, SendAndAwaitReplyRejectsWrongCommandByte)
{
        FakeCan bus;
        bus.queueRxFrame(makeFrame(0x241, 0x9A)); // right id, wrong command

        CanFrame tx = makeFrame(0x141, 0xA2);
        CanFrame rx{};
        EXPECT_FALSE(ungula::canbus::sendAndAwaitReply(
            bus, tx, /*expectedReplyId=*/0x241, /*expectedCmd=*/0xA2, rx));
}

TEST(BusHelpers, SendAndAwaitReplyFailsIfNoFrame)
{
        FakeCan bus; // no queued reply
        CanFrame tx = makeFrame(0x141, 0xA2);
        CanFrame rx{};
        EXPECT_FALSE(ungula::canbus::sendAndAwaitReply(
            bus, tx, /*expectedReplyId=*/0x241, /*expectedCmd=*/0xA2, rx));
}

TEST(BusHelpers, SendAndAwaitReplyFailsIfSendFails)
{
        FakeCan bus;
        bus.fail_next_send = true;
        bus.queueRxFrame(makeFrame(0x241, 0xA2)); // would match if send succeeded

        CanFrame tx = makeFrame(0x141, 0xA2);
        CanFrame rx{};
        EXPECT_FALSE(ungula::canbus::sendAndAwaitReply(
            bus, tx, /*expectedReplyId=*/0x241, /*expectedCmd=*/0xA2, rx));
        // Send slot wasn't recorded.
        EXPECT_EQ(bus.sent.size(), 0u);
}

// ---- scanNodes ------------------------------------------------------

TEST(BusHelpers, ScanNodesCountsResponders)
{
        FakeCan bus;
        // Pre-queue replies for ids 2 and 5 only.
        bus.queueRxFrame(makeFrame(0x240 + 1, 0x9A)); // id 1 reply
        bus.queueRxFrame(makeFrame(0x240 + 2, 0x9A)); // id 2 reply
        bus.queueRxFrame(makeFrame(0x240 + 3, 0x9A)); // id 3 reply
        bus.queueRxFrame(makeFrame(0x240 + 4, 0x9A)); // id 4 reply
        bus.queueRxFrame(makeFrame(0x240 + 5, 0x9A)); // id 5 reply

        const int n = ungula::canbus::scanNodes(
            bus, /*firstId=*/1, /*lastId=*/5,
            [](uint8_t id) { return makeFrame(0x140 + id, 0x9A); },
            [](const CanFrame &rx, uint8_t expectedId) {
                    return rx.id == 0x240u + expectedId && rx.data[0] == 0x9A;
            });

        EXPECT_EQ(n, 5);
        EXPECT_EQ(bus.sent.size(), 5u); // one probe per id
}

TEST(BusHelpers, ScanNodesIgnoresMismatchedReplies)
{
        FakeCan bus;
        // Wrong id in the reply for probe 1.
        bus.queueRxFrame(makeFrame(0x999, 0x9A));
        // Wrong command in the reply for probe 2.
        bus.queueRxFrame(makeFrame(0x240 + 2, 0x55));
        // Correct reply for probe 3.
        bus.queueRxFrame(makeFrame(0x240 + 3, 0x9A));

        const int n = ungula::canbus::scanNodes(
            bus, 1, 3,
            [](uint8_t id) { return makeFrame(0x140 + id, 0x9A); },
            [](const CanFrame &rx, uint8_t expectedId) {
                    return rx.id == 0x240u + expectedId && rx.data[0] == 0x9A;
            });

        EXPECT_EQ(n, 1);
}

TEST(BusHelpers, ScanNodesRejectsBackwardRange)
{
        FakeCan bus;
        const int n = ungula::canbus::scanNodes(
            bus, /*firstId=*/10, /*lastId=*/3,
            [](uint8_t id) { return makeFrame(0x140 + id, 0x9A); },
            [](const CanFrame &, uint8_t) { return true; });
        EXPECT_EQ(n, 0);
        EXPECT_EQ(bus.sent.size(), 0u);
}

TEST(BusHelpers, ScanNodesRejectsZeroFirstId)
{
        FakeCan bus;
        const int n = ungula::canbus::scanNodes(
            bus, /*firstId=*/0, /*lastId=*/5,
            [](uint8_t id) { return makeFrame(0x140 + id, 0x9A); },
            [](const CanFrame &, uint8_t) { return true; });
        EXPECT_EQ(n, 0);
}
