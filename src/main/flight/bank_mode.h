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

// BANK mode: fly a multirotor like a car on a banked track. See flight/bank_mode.c.
// OPEN: LED strip colour, OSD warning, blackbox fields and the Configurator modes tab entry (MSP permanentId 59).

#ifdef USE_BANK_MODE

typedef struct bankInputs_s {
    float bank;                 // Bank stick, -1..1, positive right
    float speed;                // Speed stick, 0..1
    float reverse;              // throttle stick below min_check, 0..1, 1 at the bottom
    float lift;                 // Lift stick, 0..1, 0.5 = 1 g
    bool armed;
    float accX;                 // specific force, body frame (X forward, Y left, Z up), g
    float accY;
    float accZ;
    float gyroPitchDps;         // body rates, positive nose-down
    float gyroYawDps;           // positive counter-clockwise
    float measuredPitchDeg;     // positive nose-down
    float measuredRollDeg;      // positive right
    float dt;                   // s, 0 or longer than BANK_MAX_DT_S restarts the filters
    bool motorsSaturated;       // the mixer could not give the last collective
} bankInputs_t;

typedef struct bankOutputs_s {
    float rollDeg;              // the attitude the thrust is turned towards, positive right
    float pitchDeg;             // positive nose-down
    float rateDps[3];           // body rate setpoints: roll positive right, pitch nose-down, yaw counter-clockwise
    float yawRateDps;           // the heading's turn rate, positive counter-clockwise
    float collective;           // 0..1
    bool throttleLow;           // Lift stick at the bottom, or no collective
} bankOutputs_t;

// The airframe the mode starts from, the simulator's five inch.
#ifndef BANK_DRAG_RATE_DEFAULT
#define BANK_DRAG_RATE_DEFAULT          0.40f   // 1/s, learned in flight from here
#endif
// The thrust model's full-collective thrust / weight, high on purpose: the thrust trim corrects a value too high fully,
// one too low only partly.
#ifndef BANK_THRUST_TO_WEIGHT
#define BANK_THRUST_TO_WEIGHT           5.70f
#endif

// Learned in flight and kept until power-off, through disarming and BANK switching off.
typedef struct bankLearned_s {
    float dragRate;             // D, 1/s: rotor drag at 1 g of thrust (g of drag) = D * airspeed / g; 0 = not started
    float dragNum;              // least squares sums, slowly forgotten: D = dragNum / dragDen
    float dragDen;
    float force[2];             // horizontal specific force in the heading frame (forward, left), filtered as the drag is, g
    float drag[2];              // the rotor drag read as a level velocity, filtered, g of drag
    float windowForce[2];       // the force integrated since the window started, g s
    float windowDrag[2];        // the drag at the window's start
    float windowTime;           // s
    bool windowOk;              // the conditions for learning have held since the window started
} bankLearned_t;

typedef struct bankState_s {
    float velForward;           // velocity in the heading frame, g of drag
    float velLeft;
    float velUp;
    float dragLift;             // vertical part of the drag in the rotor plane, filtered, g
    float verticalForce;        // measured vertical specific force, filtered, g
    float climbRate;            // vertical speed from the vertical force alone, slowly forgotten, g s
    float liftIntegral;         // vertical thrust the vertical force loop adds, g
    float collective;           // previous update's collective
    float thrustTrim;           // factor on the modelled thrust, 1 = as BANK_THRUST_TO_WEIGHT says
    float budgetTrim;           // the trim, filtered slowly, for the thrust the vector is shared out of
    float trimModelThrust;      // modelled thrust, lagged like the motors, g
    float trimInflow;           // inflow through the props the trim was measured at, g of drag
    bool tiltLocked;            // engaged with the throttle low, no tilt until the sticks are centred
    bool reverseReady;          // the Speed stick has left the reverse zone since arming
    timeUs_t lastUpdateUs;
} bankState_t;

void bankModeUpdate(timeUs_t currentTimeUs);
bool bankModeIsActive(void);
float bankModeGetAngleTarget(int axis);     // FD_ROLL or FD_PITCH, degrees
float bankModeGetRate(int axis);            // body rate setpoint, degrees per second
float bankModeGetCollective(void);          // 0..1
float bankModeGetThrustToWeight(void);
bool bankModeIsThrottleLow(void);           // replaces calculateThrottleStatus() == THROTTLE_LOW under BANK

#endif // USE_BANK_MODE
