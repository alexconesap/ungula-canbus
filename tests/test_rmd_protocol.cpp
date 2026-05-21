// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include "ungula/canbus/devices/rmd/protocol.h"

namespace rmd = ungula::canbus::rmd;

// ---- Pack / unpack round-trip ---------------------------------------

TEST(RmdProtocol, PackLe16RoundTrip)
{
        uint8_t buf[2] = {};
        rmd::pack_le16(buf, 0xBEEFu);
        EXPECT_EQ(buf[0], 0xEFu);
        EXPECT_EQ(buf[1], 0xBEu);
        EXPECT_EQ(rmd::unpack_le16(buf), 0xBEEFu);
}

TEST(RmdProtocol, PackLe32PositiveValue)
{
        uint8_t buf[4] = {};
        rmd::pack_le32(buf, 0x12345678);
        EXPECT_EQ(buf[0], 0x78u);
        EXPECT_EQ(buf[1], 0x56u);
        EXPECT_EQ(buf[2], 0x34u);
        EXPECT_EQ(buf[3], 0x12u);
        EXPECT_EQ(rmd::unpack_le32(buf), 0x12345678);
}

TEST(RmdProtocol, PackLe32NegativeValueTwosComplement)
{
        uint8_t buf[4] = {};
        rmd::pack_le32(buf, -1);
        EXPECT_EQ(buf[0], 0xFFu);
        EXPECT_EQ(buf[1], 0xFFu);
        EXPECT_EQ(buf[2], 0xFFu);
        EXPECT_EQ(buf[3], 0xFFu);
        EXPECT_EQ(rmd::unpack_le32(buf), -1);
}

TEST(RmdProtocol, PackLe32BoundaryValues)
{
        uint8_t buf[4] = {};
        rmd::pack_le32(buf, INT32_MIN);
        EXPECT_EQ(rmd::unpack_le32(buf), INT32_MIN);
        rmd::pack_le32(buf, INT32_MAX);
        EXPECT_EQ(rmd::unpack_le32(buf), INT32_MAX);
        rmd::pack_le32(buf, 0);
        EXPECT_EQ(rmd::unpack_le32(buf), 0);
}

// ---- Frame builder --------------------------------------------------

TEST(RmdProtocol, MakeTxAddressesMotorAndZeroesPayload)
{
        const auto f = rmd::makeTx(/*motor_id=*/3, rmd::CMD_SPEED_LOOP);
        EXPECT_EQ(f.id, rmd::TX_BASE + 3u);
        EXPECT_FALSE(f.extendedId);
        EXPECT_EQ(f.dlc, 8u);
        EXPECT_EQ(f.data[0], rmd::CMD_SPEED_LOOP);
        for (int i = 1; i < 8; ++i) {
                EXPECT_EQ(f.data[i], 0u) << "byte " << i;
        }
}

// ---- Addressing constants ------------------------------------------

TEST(RmdProtocol, AddressingConstantsMatchV3Spec)
{
        EXPECT_EQ(rmd::TX_BASE,      0x140u);
        EXPECT_EQ(rmd::RX_BASE,      0x240u);
        EXPECT_EQ(rmd::TX_BROADCAST, 0x300u);
        EXPECT_EQ(rmd::MOTOR_ID_MIN, 1u);
        EXPECT_EQ(rmd::MOTOR_ID_MAX, 32u);
}

TEST(RmdProtocol, CommandBytesMatchV3Spec)
{
        // Spot-check the bytes hosts most often inspect.
        EXPECT_EQ(rmd::CMD_SHUTDOWN,        0x80);
        EXPECT_EQ(rmd::CMD_STOP,            0x81);
        EXPECT_EQ(rmd::CMD_WAKE_UP,         0x88);
        EXPECT_EQ(rmd::CMD_READ_STATUS_1,   0x9A);
        EXPECT_EQ(rmd::CMD_SET_NODE_ID,     0x79);
        EXPECT_EQ(rmd::CMD_SPEED_LOOP,      0xA2);
        EXPECT_EQ(rmd::CMD_POSITION_ABS,    0xA4);
        EXPECT_EQ(rmd::CMD_POSITION_INCR,   0xA8);
        EXPECT_EQ(rmd::CMD_READ_POSITION,   0x60);
        EXPECT_EQ(rmd::CMD_SET_ZERO_OFFSET, 0x64);
        EXPECT_EQ(rmd::CMD_READ_RUNTIME_MS, 0xB1);
}
