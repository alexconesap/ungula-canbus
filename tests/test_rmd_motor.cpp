// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include "fakes/fake_can.h"
#include "ungula/canbus/devices/rmd/motor.h"

namespace rmd = ungula::canbus::rmd;
using ungula::canbus::tests::FakeCan;
using ungula::hal::can::CanFrame;

// ---- Speed loop -----------------------------------------------------

TEST(RmdMotor, SendSpeedEmitsA2FrameWithLeSpeed)
{
        FakeCan bus;
        ASSERT_TRUE(rmd::sendSpeed(bus, /*motor_id=*/3, /*centidps=*/12345));

        ASSERT_EQ(bus.sent.size(), 1u);
        const auto &f = bus.sent[0];
        EXPECT_EQ(f.id,         rmd::TX_BASE + 3u);
        EXPECT_EQ(f.dlc,        8u);
        EXPECT_EQ(f.data[0],    rmd::CMD_SPEED_LOOP);
        // bytes [4..7] are int32 LE 12345 = 0x00003039
        EXPECT_EQ(f.data[4], 0x39u);
        EXPECT_EQ(f.data[5], 0x30u);
        EXPECT_EQ(f.data[6], 0x00u);
        EXPECT_EQ(f.data[7], 0x00u);
}

TEST(RmdMotor, SendSpeedSignBitForBackwardDirection)
{
        FakeCan bus;
        ASSERT_TRUE(rmd::sendSpeed(bus, 1, /*centidps=*/-100));

        const auto &f = bus.sent[0];
        // -100 as int32 LE = 0xFFFFFF9C
        EXPECT_EQ(f.data[4], 0x9Cu);
        EXPECT_EQ(f.data[5], 0xFFu);
        EXPECT_EQ(f.data[6], 0xFFu);
        EXPECT_EQ(f.data[7], 0xFFu);
}

TEST(RmdMotor, SendSpeedRejectsOutOfRangeMotorId)
{
        FakeCan bus;
        EXPECT_FALSE(rmd::sendSpeed(bus, 0,  100));
        EXPECT_FALSE(rmd::sendSpeed(bus, 33, 100));
        EXPECT_EQ(bus.sent.size(), 0u);
}

TEST(RmdMotor, SendSpeedIfChangedSkipsCacheHit)
{
        FakeCan bus;
        rmd::invalidateSpeedCache(0); // clear all cached entries

        ASSERT_TRUE(rmd::sendSpeedIfChanged(bus, 1, 500));
        ASSERT_TRUE(rmd::sendSpeedIfChanged(bus, 1, 500)); // unchanged -> skip

        EXPECT_EQ(bus.sent.size(), 1u); // only the first call hit the wire
}

TEST(RmdMotor, SendSpeedIfChangedRetriesAfterFailure)
{
        FakeCan bus;
        rmd::invalidateSpeedCache(0);

        bus.fail_next_send = true;
        EXPECT_FALSE(rmd::sendSpeedIfChanged(bus, 1, 700));
        // Send failed; cache stays invalid, so the next call retries.
        ASSERT_TRUE(rmd::sendSpeedIfChanged(bus, 1, 700));
        ASSERT_TRUE(rmd::sendSpeedIfChanged(bus, 1, 700)); // cached now
        EXPECT_EQ(bus.sent.size(), 1u);
}

TEST(RmdMotor, InvalidateSpeedCacheZeroClearsAllEntries)
{
        FakeCan bus;
        rmd::invalidateSpeedCache(0);
        rmd::sendSpeedIfChanged(bus, 1, 100);
        rmd::sendSpeedIfChanged(bus, 2, 200);

        rmd::invalidateSpeedCache(0); // wipe everything
        bus.reset();
        ASSERT_TRUE(rmd::sendSpeedIfChanged(bus, 1, 100));
        ASSERT_TRUE(rmd::sendSpeedIfChanged(bus, 2, 200));
        EXPECT_EQ(bus.sent.size(), 2u); // both retransmitted
}

// ---- State control --------------------------------------------------

TEST(RmdMotor, StopSendsCmd81)
{
        FakeCan bus;
        ASSERT_TRUE(rmd::stop(bus, 1));
        EXPECT_EQ(bus.sent[0].data[0], rmd::CMD_STOP);
}

TEST(RmdMotor, ShutdownSendsCmd80AndInvalidatesCache)
{
        FakeCan bus;
        rmd::invalidateSpeedCache(0);
        rmd::sendSpeedIfChanged(bus, 1, 999); // cache becomes valid
        bus.reset();

        ASSERT_TRUE(rmd::shutdown(bus, 1));
        EXPECT_EQ(bus.sent[0].data[0], rmd::CMD_SHUTDOWN);

        // The cache invalidation means the next sendSpeedIfChanged
        // retransmits even with the same setpoint.
        bus.reset();
        ASSERT_TRUE(rmd::sendSpeedIfChanged(bus, 1, 999));
        EXPECT_EQ(bus.sent.size(), 1u);
}

TEST(RmdMotor, WakeUpSendsCmd88)
{
        FakeCan bus;
        ASSERT_TRUE(rmd::wakeUp(bus, 1));
        EXPECT_EQ(bus.sent[0].data[0], rmd::CMD_WAKE_UP);
}

// ---- Position moves -------------------------------------------------

TEST(RmdMotor, MoveAbsoluteEncodesSpeedAndPosition)
{
        FakeCan bus;
        ASSERT_TRUE(rmd::moveAbsolute(bus, /*motor_id=*/1,
                                       /*target_centideg=*/36000,
                                       /*max_speed_dps=*/60));
        const auto &f = bus.sent[0];
        EXPECT_EQ(f.data[0], rmd::CMD_POSITION_ABS);
        // bytes [2..3] = uint16 LE 60 = 0x003C
        EXPECT_EQ(f.data[2], 0x3Cu);
        EXPECT_EQ(f.data[3], 0x00u);
        // bytes [4..7] = int32 LE 36000 = 0x00008CA0
        EXPECT_EQ(f.data[4], 0xA0u);
        EXPECT_EQ(f.data[5], 0x8Cu);
        EXPECT_EQ(f.data[6], 0x00u);
        EXPECT_EQ(f.data[7], 0x00u);
}

TEST(RmdMotor, MoveRelativeEncodesSignedDelta)
{
        FakeCan bus;
        ASSERT_TRUE(rmd::moveRelative(bus, 1, /*delta_centideg=*/-9000,
                                        /*max_speed_dps=*/30));
        const auto &f = bus.sent[0];
        EXPECT_EQ(f.data[0], rmd::CMD_POSITION_INCR);
        EXPECT_EQ(f.data[2], 0x1Eu); // 30 LE
        EXPECT_EQ(f.data[3], 0x00u);
        // -9000 as int32 LE = 0xFFFFDCD8
        EXPECT_EQ(f.data[4], 0xD8u);
        EXPECT_EQ(f.data[5], 0xDCu);
        EXPECT_EQ(f.data[6], 0xFFu);
        EXPECT_EQ(f.data[7], 0xFFu);
}

// ---- Reads ----------------------------------------------------------

TEST(RmdMotor, ReadPositionDecodesLe32Payload)
{
        FakeCan bus;
        // Reply: 0x00 01 02 03 (header bytes), then int32 LE 0x12345678.
        ungula::hal::can::CanFrame reply{};
        reply.id      = rmd::RX_BASE + 1;
        reply.dlc     = 8;
        reply.data[0] = rmd::CMD_READ_POSITION;
        reply.data[4] = 0x78;
        reply.data[5] = 0x56;
        reply.data[6] = 0x34;
        reply.data[7] = 0x12;
        bus.queueRxFrame(reply);

        int32_t pos = 0;
        ASSERT_TRUE(rmd::readPosition(bus, 1, &pos));
        EXPECT_EQ(pos, 0x12345678);
}

TEST(RmdMotor, ReadPositionFailsOnWrongReplyId)
{
        FakeCan bus;
        ungula::hal::can::CanFrame reply{};
        reply.id      = 0x999; // wrong
        reply.dlc     = 8;
        reply.data[0] = rmd::CMD_READ_POSITION;
        bus.queueRxFrame(reply);

        int32_t pos = 999;
        EXPECT_FALSE(rmd::readPosition(bus, 1, &pos));
}

TEST(RmdMotor, ReadStatusRawCopiesEightBytes)
{
        FakeCan bus;
        ungula::hal::can::CanFrame reply{};
        reply.id      = rmd::RX_BASE + 2;
        reply.dlc     = 8;
        reply.data[0] = rmd::CMD_READ_STATUS_1;
        reply.data[1] = 0x25; // temperature, decoded by host
        reply.data[3] = 0xE0; // bus voltage low byte
        reply.data[4] = 0x00; // bus voltage high byte
        reply.data[6] = 0x00;
        reply.data[7] = 0x00;
        bus.queueRxFrame(reply);

        uint8_t buf[8] = {};
        ASSERT_TRUE(rmd::readStatusRaw(bus, 2, buf));
        EXPECT_EQ(buf[0], rmd::CMD_READ_STATUS_1);
        EXPECT_EQ(buf[1], 0x25u);
        EXPECT_EQ(buf[3], 0xE0u);
}

TEST(RmdMotor, ReadSystemRuntimeDecodesUptime)
{
        FakeCan bus;
        ungula::hal::can::CanFrame reply{};
        reply.id      = rmd::RX_BASE + 1;
        reply.dlc     = 8;
        reply.data[0] = rmd::CMD_READ_RUNTIME_MS;
        // bytes [4..7] = uint32 LE 60000 (one minute)
        reply.data[4] = 0x60;
        reply.data[5] = 0xEA;
        reply.data[6] = 0x00;
        reply.data[7] = 0x00;
        bus.queueRxFrame(reply);

        uint8_t buf[8] = {};
        ASSERT_TRUE(rmd::readSystemRuntime(bus, 1, buf));
        const uint32_t uptime = static_cast<uint32_t>(buf[4]) |
                                (static_cast<uint32_t>(buf[5]) << 8) |
                                (static_cast<uint32_t>(buf[6]) << 16) |
                                (static_cast<uint32_t>(buf[7]) << 24);
        EXPECT_EQ(uptime, 60000u);
}

// ---- Zero set -------------------------------------------------------

TEST(RmdMotor, SetCurrentPositionAsZeroSendsCmd64)
{
        FakeCan bus;
        ASSERT_TRUE(rmd::setCurrentPositionAsZero(bus, 1));
        EXPECT_EQ(bus.sent[0].data[0], rmd::CMD_SET_ZERO_OFFSET);
}

// ---- Discovery + addressing -----------------------------------------

TEST(RmdMotor, SetNodeIdSendsBroadcastWithNewIdInByte7)
{
        FakeCan bus;
        ASSERT_TRUE(rmd::setNodeId(bus, /*new_node_id=*/7));
        const auto &f = bus.sent[0];
        EXPECT_EQ(f.id,      rmd::TX_BROADCAST);
        EXPECT_EQ(f.data[0], rmd::CMD_SET_NODE_ID);
        EXPECT_EQ(f.data[2], 0x00u);   // 0 = write
        EXPECT_EQ(f.data[7], 7u);
}

TEST(RmdMotor, SetNodeIdRejectsOutOfRange)
{
        FakeCan bus;
        EXPECT_FALSE(rmd::setNodeId(bus, 0));
        EXPECT_FALSE(rmd::setNodeId(bus, 33));
        EXPECT_EQ(bus.sent.size(), 0u);
}

TEST(RmdMotor, ScanNodesCountsOnlyMatchingReplies)
{
        FakeCan bus;

        // Per-probe reply mapping: id 5 replies cleanly, id 7 replies
        // with a wrong command byte (must not count), all others
        // silent. `scanNodes` drains the RX queue first, so the
        // pre-queue path can't be used here - the reply must land
        // right after the probe it answers, which is exactly what
        // `reply_for_tx_id` does.
        ungula::hal::can::CanFrame reply5{};
        reply5.id      = rmd::RX_BASE + 5;
        reply5.dlc     = 8;
        reply5.data[0] = rmd::CMD_READ_STATUS_1;
        bus.reply_for_tx_id[rmd::TX_BASE + 5] = reply5;

        ungula::hal::can::CanFrame reply7_bad{};
        reply7_bad.id      = rmd::RX_BASE + 7;
        reply7_bad.dlc     = 8;
        reply7_bad.data[0] = 0x00; // wrong command echo
        bus.reply_for_tx_id[rmd::TX_BASE + 7] = reply7_bad;

        const int n = rmd::scanNodes(bus);
        EXPECT_EQ(n, 1);
        EXPECT_EQ(bus.sent.size(), 32u);
}
