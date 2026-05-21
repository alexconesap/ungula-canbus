// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#ifndef __cplusplus
#error UngulaCanbus requires a C++ compiler
#endif

// UngulaCanbus - CAN-bus service layer + per-brand wire protocols.
// Sits on top of UngulaHal's classic-CAN abstraction (`ICan`) and
// underneath any application code that wants to talk to a CAN
// device. Hosts use the per-brand headers directly (see below);
// they never deal with frame layouts or platform specifics.

// Hardware abstraction the lib builds on. Hosts construct a
// concrete `lib_hal::can::Can` (or any other `ICan` implementation)
// and hand it to the device-protocol calls.
#include "ungula/hal/can/can.h"
#include "ungula/hal/can/can_frame.h"
#include "ungula/hal/can/i_can.h"

// Generic bus services (protocol-agnostic): node discovery, drain,
// send-and-await. Used by per-brand device protocols and by hosts
// that talk directly to the bus.
#include "ungula/canbus/bus_helpers.h"

// Per-brand device protocols. Hosts include only the brand they
// need; each protocol file pulls in its own `<brand>/protocol.h`
// transitively.
#include "ungula/canbus/devices/rmd/motor.h"
