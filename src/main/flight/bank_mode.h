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
#include <stdint.h>

#include "common/time.h"

/*
 * BANK mode: fly a multirotor like a car on a banked track. See flight/bank_mode.c for the behaviour.
 *
 * Where it plugs in:
 *   fc/core.c             sets BANK_MODE (an alternative to ANGLE/HORIZON) and calls bankModeUpdate()
 *   flight/pid.c          takes the roll/pitch angle targets from bankModeGetAngleTarget()
 *   fc/rc.c               takes the yaw setpoint from bankModeGetYawRate()
 *   flight/mixer.c        takes the throttle from bankModeGetCollective()
 *   osd/osd_elements.c, telemetry/{ltm,smartport,ibus_shared}.c   mode reporting
 *
 * Not done yet: LED strip mode colour, OSD warning, blackbox fields, and the Betaflight Configurator
 * modes tab entry (MSP permanentId 59).
 */

#ifdef USE_BANK_MODE

// Roll, pitch and yaw stick inputs are -1..1: roll to the right, pitch forward, yaw counter-clockwise.
// The vertical input is the throttle stick position, 0 at the bottom (no lift) to 1 at the top.
typedef struct bankInputs_s {
    float roll;
    float pitch;
    float yaw;
    float vertical;
    bool throttleLow;
    float dragX;                // filtered specific force along body X (forward), g
    float dragY;                // filtered specific force along body Y (left), g: the "ball"
    float measuredPitchDeg;     // measured attitude, same signs as the targets: pitch positive nose-down
    float measuredRollDeg;      // measured attitude: roll positive to the right
    float dt;                   // seconds since the previous update
} bankInputs_t;

typedef struct bankOutputs_s {
    float rollDeg;              // roll angle target, positive to the right
    float pitchDeg;             // pitch angle target, positive nose-down (forward)
    float yawRateDps;           // yaw rate setpoint, positive counter-clockwise
    float collective;           // fraction of maximum thrust, 0..1
} bankOutputs_t;

typedef struct bankState_s {
    float headCheckOffset;      // radians, positive = nose counter-clockwise from the travel direction
    float dragX;
    float dragY;
    float ballIntegrator;       // yaw rate held by the ball integrator, degrees per second
    bool filtersValid;
    bool started;               // the first update since the mode was engaged has been seen
    bool tiltLocked;            // engaged with the throttle low: no tilt until the right stick is centred
    timeUs_t lastUpdateUs;
} bankState_t;

void bankModeUpdate(timeUs_t currentTimeUs);
bool bankModeIsActive(void);
float bankModeGetAngleTarget(int axis);     // FD_ROLL or FD_PITCH, degrees
float bankModeGetYawRate(void);             // degrees per second
float bankModeGetCollective(void);          // 0..1 of maximum thrust

#endif // USE_BANK_MODE
