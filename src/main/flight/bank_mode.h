/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

#include "common/time.h"

/*
 * BANK mode: fly a multirotor like a car on a banked track.
 * See flight/bank_mode.c for the intended behaviour. This is currently a skeleton.
 *
 * Files still to be touched for the full feature (all gated by USE_BANK_MODE):
 *   fc/rc.c               stick -> force-vector setpoint shaping (cf. the HEADFREE_MODE block)
 *   flight/imu.c          travel heading and head-check yaw offset (cf. headfree attitude handling)
 *   flight/pid.c          attitude / yaw-rate targets from bank mode
 *   flight/mixer.c        tilt-compensated collective, thrust saturation
 *   fc/core.c             arming / failsafe / mode-priority interaction
 *   osd/osd_elements.c    flight mode text
 *   osd/osd_warnings.c    mode warning, cf. HEADFREE
 *   io/ledstrip.c/.h      LED_MODE_BANK
 *   telemetry/ltm.c, smartport.c, ibus_shared.c   flight mode reporting
 *   blackbox/             log bank state (travel heading, commanded force)
 *   cli/settings.c, pg/bank_mode.c/.h   remaining config (max bank angle, head-check range/rate, deadbands)
 *   Betaflight Configurator: BANK mode in the modes tab (MSP permanentId 59)
 */

#ifdef USE_BANK_MODE

void bankModeUpdate(timeUs_t currentTimeUs);
bool bankModeIsActive(void);

#endif // USE_BANK_MODE
