/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief PX4 custom flight modes
 * @file px4_custom_mode.hpp
 *
 * @author Thiago Marques de Oliveira
 * @date April, 2026
 */


#pragma once

#include <stdint.h>

namespace px4_custom_mode
{

// This enum was copied from PX4 firmware repo (src/modules/commander/px4_custom_mode.hpp)
enum PX4_CUSTOM_MAIN_MODE : uint8_t {
	PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
	PX4_CUSTOM_MAIN_MODE_ALTCTL,
	PX4_CUSTOM_MAIN_MODE_POSCTL,
	PX4_CUSTOM_MAIN_MODE_AUTO,
	PX4_CUSTOM_MAIN_MODE_ACRO,
	PX4_CUSTOM_MAIN_MODE_OFFBOARD,
	PX4_CUSTOM_MAIN_MODE_STABILIZED,
	PX4_CUSTOM_MAIN_MODE_RATTITUDE_LEGACY,
	PX4_CUSTOM_MAIN_MODE_SIMPLE, /* unused, but reserved for future use */
	PX4_CUSTOM_MAIN_MODE_TERMINATION
};

}
