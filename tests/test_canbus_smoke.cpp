// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.
//
// Smoke test. Proves the lib_canbus include chain compiles and that
// the lib_hal `Can` class satisfies the `ICan` interface on the host
// build. Real coverage lands in step 3 (bus_helpers + RMD protocol).

#include <gtest/gtest.h>

#include "ungula/hal/can/can.h"
#include "ungula/hal/can/can_frame.h"
#include "ungula/hal/can/i_can.h"

TEST(LibCanbusSmoke, CanImplementsICan)
{
        ungula::hal::can::Can concrete(0);

        // Polymorphic handle - this is the shape every lib_canbus
        // protocol takes its bus argument as.
        ungula::hal::can::ICan &bus = concrete;

        ungula::hal::can::CanFrame f{};
        f.id = 0x123;
        f.dlc = 0;

        // Host stub: begin() is on the concrete class. After install
        // succeeds, send / receive / isBusOff / recoverFromBusOff /
        // stop are all reachable through the interface.
        ASSERT_TRUE(concrete.begin(/*tx=*/21, /*rx=*/22,
                                   ungula::hal::can::BITRATE_500K));
        EXPECT_TRUE(bus.send(f));
        EXPECT_FALSE(bus.isBusOff());
        EXPECT_TRUE(bus.recoverFromBusOff());
        EXPECT_TRUE(bus.stop());
}

TEST(LibCanbusSmoke, BitrateConstantsAreAccessible)
{
        // Sanity check that the constants moved into can_frame.h are
        // visible after including lib_canbus's umbrella entry points.
        EXPECT_EQ(ungula::hal::can::BITRATE_1M,   1'000'000u);
        EXPECT_EQ(ungula::hal::can::BITRATE_500K, 500'000u);
        EXPECT_EQ(ungula::hal::can::BITRATE_125K, 125'000u);
}
