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

#include "platform.h"

#ifdef USE_BANK_MODE

#include <stdbool.h>
#include <stdint.h>

#include "common/utils.h"

#include "fc/runtime_config.h"

#include "bank_mode.h"

/*
 * BANK mode, intended behaviour (pseudocode, not implemented yet).
 *
 * Three control axes are coupled down to two, with a spare stick axis for look-around.
 * Attitude only (gyro/ACC): no GPS or baro is required.
 *
 * Stick mapping (mode 2):
 *   throttle  -> vertical force command Fz (not raw throttle). Centre = thrust balances weight.
 *   yaw       -> head check: nose offset from the travel heading, springs back to 0 at centre.
 *   pitch     -> forward/backward force command Fx, realised as tilt theta plus collective.
 *   roll      -> bank angle phi, i.e. a turn in the horizontal plane.
 *
 * State:
 *   travelHeading      virtual course reference, similar in spirit to headfree
 *   headCheckOffset    nose yaw relative to travelHeading; must not change the velocity direction
 *   m*g                weight, expressed through bank_thrust_to_weight (T/W) so that
 *                      hover collective = 1 / T/W
 *
 * Each update, while BANK_MODE is active:
 *
 *   Fx = shape(pitchStick)                  // OPEN: scaling, reverse behaviour, deadband
 *   Fz = shape(throttleStick)               // OPEN: scaling, deadband around neutral
 *   phi = rollStick * maxBankAngle          // OPEN: max bank angle
 *   theta = atan2(Fx, m*g + Fz)             // tilt that gives forward force Fx
 *                                           // at vertical force Fz
 *
 *   // thrust saturation, in priority order: Fz first, then bank (phi), forward force (Fx) last
 *   Tmax = (T/W) * m*g
 *   Fz = min(Fz, Tmax - m*g)                // no tilt budget left at all: Fz itself is capped
 *   cosTiltMin = (m*g + Fz) / Tmax          // T <= Tmax  <=>  cos(theta) * cos(phi) >= cosTiltMin
 *   // the budget is the tilt left over once Fz is satisfied; closed form, no iteration
 *   phi = min(phi, acos(cosTiltMin))        // bank takes what it needs from the budget first
 *   theta = min(theta, acos(cosTiltMin / cos(phi)))   // forward tilt gets whatever remains
 *   Fx = (m*g + Fz) * tan(theta)            // clipped forward force; Fz is never touched
 *   // a hard turn at the limit therefore leaves little or no forward force, and the
 *   // vertical force stays consistent; the limits are continuous in the stick inputs
 *   // OPEN: whether to filter the limits, to avoid abrupt changes if Tmax drops (battery sag)
 *
 *   // collective is always computed from the (limited) tilt, never taken from the stick
 *   T = (m*g + Fz) / (cos(theta) * cos(phi))    // now guaranteed <= Tmax
 *
 *   // coordinated turn: tan(phi) = v * omega / g, so the heading rate follows the bank
 *   // use the limited phi, so a clipped bank also gives a correspondingly slower turn
 *   omega = g * tan(phi) / v                // v from the forward-force model; OPEN: low-speed limit
 *   travelHeading += omega * dt
 *
 *   // head check: yaw the nose away from travel without changing the velocity direction
 *   headCheckOffset = yawStick * maxHeadCheck, rate-limited, returning to 0 at a set rate  // OPEN
 *   noseYawTarget = travelHeading + headCheckOffset
 *
 *   -> setpoints: attitude (theta, phi), yaw (noseYawTarget), collective T handed to the mixer.
 *
 * OPEN: priority against ANGLE/HORIZON/HEADFREE/ALT_HOLD/POS_HOLD, and arming/failsafe
 * behaviour (failsafe and GPS rescue already take over in fc/core.c).
 */

static bool bankModeActive = false;

void bankModeUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // skeleton: no control is applied yet, only mirror the mode flag
    bankModeActive = FLIGHT_MODE(BANK_MODE);
}

bool bankModeIsActive(void)
{
    return bankModeActive;
}

#endif // USE_BANK_MODE
