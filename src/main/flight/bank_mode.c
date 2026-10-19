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

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"

#include "pg/bank_mode.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "bank_mode.h"

/*
 * BANK mode: fly a multirotor like a car on a banked track, as hard as acro can.
 *
 * Attitude only (gyro/ACC), no GPS or baro. Forces are in g (weight = 1). Body axes are X forward, Y left, Z up; roll
 * is positive to the right, pitch positive nose-down and yaw positive counter-clockwise. The heading frame is the
 * body frame levelled (forward and left along the nose's heading, up); the travel frame is the heading frame turned
 * onto the direction of travel.
 *
 * Sticks (mode 2):
 *   throttle  Speed: the lower half is a speed target, 0 to bank_mid_speed, with a stop band at the bottom; the upper
 *             half a forward force up to what the thrust allows. Below min_check a small reverse, once the stick has
 *             been raised since arming.
 *   pitch     Lift: BANK_LIFT_FLOOR at the bottom, 1 g at the centre, all the thrust at the top
 *   yaw       Bank: the bank angle of the thrust, up to BANK_MAX_BANK_DEG; at zero Speed a yaw rate instead
 *   roll      unused
 *
 * One thrust vector, set in the travel frame (x forward along the track, r to the right, z up) and limited to the
 * thrust there is, in priority order:
 *   1. the corner force: near full Bank the Speed stick gives way to a speed loop towards the corner speed, at which
 *      full Bank turns the track at bank_corner_rate. It brakes a fast craft before it banks, so a hard turn bleeds
 *      speed to tighten, and holds the speed of a slow one, so a held full Bank stays a turn.
 *   2. Bank: r = z * tan(bank). Where that is more than the thrust left, Lift gives way, down to BANK_LIFT_YIELD of
 *      it, so full Bank is nearly all the sideways force there is, as a flat-out acro turn, without a spiral dive.
 *   3. Speed: x, the forward force or brake of the Speed stick, from what is left.
 * The vector is turned from the travel frame into the heading frame by the slip, so where the nose points never
 * changes the force on the track. The mode flies on rates, as acro: the thrust axis is turned towards the vector about
 * the axis between them, and the heading's turn rate is shared out over the body axes, so no Euler angle is held and
 * a bank near 90 degrees is no different from any other.
 *
 * Velocity is estimated in the heading frame, in g of drag (the rotor drag at 1 g of thrust), so it needs no m/s
 * calibration: the accelerometer integrated in three axes (dv/dt = bank_drag_rate * (specific force - gravity)), and
 * corrected in the rotor plane by the rotor drag the accelerometer measures there, so a climb or a sink across tilted
 * rotors is not read as speed or slip. The same bank_drag_rate turns a sideways force into a turn rate,
 * omega = bank_drag_rate * force / speed, which the yaw feeds forward; the slip, the angle from the nose to the track,
 * closes the loop.
 *
 * Lift is held by the collective at the measured tilt, from a thrust model that the accelerometer trims along the
 * thrust axis, less the lift the rotor drag gives pitched at speed, plus an integral on the measured vertical force
 * for what both miss. The force budget follows the trim slowly, so near the edge of the envelope Lift keeps its
 * thrust and the forward tilt gives way.
 *
 * Limitations:
 *   - the drag only sees speed through the rotors: little signal at low speed, and accelerometer bias reads as speed
 *   - the vertical velocity is only seen through the drag while tilted, and level it decays to zero
 *   - Lift is a force: nothing holds the height or the vertical speed
 */

#define BANK_MIN_DT_S           0.0005f
#define BANK_MAX_DT_S           0.2f    // longer gaps restart the filters
#define BANK_FALLBACK_DT_S      0.01f
#define BANK_STICK_DEADBAND     0.05f
#define BANK_THRUST_EXPONENT    1.5f    // thrust ~ collective ^ 1.5
#define BANK_THRUST_SHARE       0.95f   // share of the thrust the vector may use, the rest is the attitude loop's
#define BANK_MAX_BANK_DEG       85.0f   // the bank of a full Bank stick, when the thrust allows it
#define BANK_MIN_LIFT           0.5f    // g, the lowest lift the pitch is computed for
#define BANK_LIFT_FLOOR         0.2f    // g, Lift at the bottom of the stick: enough thrust to steer while dropping
#define BANK_MAX_BRAKE_DEG      60.0f
#define BANK_BRAKE_STICK        (1.0f / 3.0f)   // below this Speed stick a craft that is too fast is braked
#define BANK_REVERSE_STICK      0.05f   // the reverse zone carries the Speed stick this far below 0
#define BANK_STOP_STICK         0.10f   // Speed stick above min_check that still asks for no speed
#define BANK_SPEED_KP           2.0f    // g per g of drag
#define BANK_SPEED_DEADBAND     0.02f   // g of drag
#define BANK_GROUND_SPEED       0.05f   // g of drag, faster than this the craft is flying
#define BANK_CORNER_START       0.6f    // Bank stick where the corner speed starts to take over
#define BANK_CORNER_FULL        0.85f   // and where it has
#define BANK_LIFT_YIELD         0.8f    // share of Lift a full Bank may not take
#define BANK_VEL_DRAG_GAIN      2.0f    // 1/s, how fast the velocity estimate follows the measured drag
#define BANK_VEL_UP_LEAK        0.2f    // 1/s, the vertical velocity, which the drag can't see level, decays
#define BANK_DRAG_MIN_THRUST    0.25f   // g
#define BANK_LEVEL_MIN_COS      0.087f  // cos(85 deg)
#define BANK_TILT_GAIN          20.0f   // 1/s, tilt rate per tilt error
#define BANK_TILT_RATE          900.0f  // deg/s
#define BANK_YAW_MAX_RATE       1000.0f // deg/s; in a steep bank most of it is pitch rate
#define BANK_SLIP_GAIN          10.0f   // yaw rate per slip, 1/s
#define BANK_TURN_MIN_SPEED     0.1f    // g of drag, the turn fades out below this
#define BANK_HOVER_YAW_RATE     400.0f  // deg/s at full Bank stick and zero Speed
#define BANK_DRAG_LIFT_HZ       5.0f    // filter on the vertical part of the drag and on the vertical force
#define BANK_LIFT_KI            3.0f    // 1/s, the vertical force loop's integral
#define BANK_LIFT_I_MAX         1.0f    // g
#define BANK_LIFT_I_LEAK_S      1.0f    // where it doesn't integrate, it leaks back to 0
#define BANK_TRIM_MIN_THRUST    0.3f    // g, no trim measurement below this modelled thrust
#define BANK_TRIM_FILTER_HZ     3.0f
#define BANK_TRIM_LAG_HZ        10.0f   // motor and accelerometer lag, applied to the modelled thrust
#define BANK_TRIM_MIN           0.05f   // fast forward flight unloads the props almost entirely
#define BANK_TRIM_MAX           2.0f
#define BANK_TRIM_LEAK_S        1.0f
#define BANK_TRIM_INFLOW_FLOOR  0.1f    // g of drag
#define BANK_BUDGET_HZ          1.0f    // the thrust the vector is shared out of follows the trim this slowly

static bankState_t bankState;

static FAST_DATA_ZERO_INIT bankOutputs_t bankOutputs;     // read by the PID loop and the mixer

// Removes the deadband and rescales so full deflection still gives 1.
STATIC_UNIT_TESTED float bankShapeStick(float deflection)
{
    const float d = constrainf(deflection, -1.0f, 1.0f);
    const float magnitude = fabsf(d);
    if (magnitude <= BANK_STICK_DEADBAND) {
        return 0.0f;
    }
    const float shaped = (magnitude - BANK_STICK_DEADBAND) / (1.0f - BANK_STICK_DEADBAND);
    return d < 0.0f ? -shaped : shaped;
}

// Lift in g for the Lift stick (0..1): linear from BANK_LIFT_FLOOR to 1 g below the centre, then
// 1 + (1 - floor) * x + (T/W - 2 + floor) * x^3 up to T/W, so both halves start at the same slope.
STATIC_UNIT_TESTED float bankLiftFromStick(float stick, float thrustToWeight)
{
    const float s = constrainf(stick, 0.0f, 1.0f);
    const float slope = 1.0f - BANK_LIFT_FLOOR;
    if (s <= 0.5f) {
        return BANK_LIFT_FLOOR + 2.0f * s * slope;
    }
    const float x = 2.0f * s - 1.0f;
    const float tw = fmaxf(thrustToWeight, 1.0f);
    if (tw <= 2.0f - BANK_LIFT_FLOOR) {
        return 1.0f + x * (tw - 1.0f);     // no room for the curve
    }
    return 1.0f + slope * x + (tw - 1.0f - slope) * x * x * x;
}

// thrust (g) = thrustToWeight * collective ^ BANK_THRUST_EXPONENT
STATIC_UNIT_TESTED float bankThrustFromCollective(float collective, float thrustToWeight)
{
    return thrustToWeight * powf(fmaxf(collective, 0.0f), BANK_THRUST_EXPONENT);
}

STATIC_UNIT_TESTED float bankCollectiveFromThrust(float thrust, float thrustToWeight)
{
    return powf(constrainf(thrust / fmaxf(thrustToWeight, 0.1f), 0.0f, 1.0f), 1.0f / BANK_THRUST_EXPONENT);
}

// Rotates a horizontal vector (forward, left) counter-clockwise by angle.
STATIC_UNIT_TESTED void bankRotate(float *forward, float *left, float angle)
{
    const float c = cosf(angle);
    const float s = sinf(angle);
    const float x = *forward;
    *forward = x * c - *left * s;
    *left = x * s + *left * c;
}

// A body-frame vector in the heading frame, for ZYX angles (roll positive right, pitch positive nose-down).
STATIC_UNIT_TESTED void bankBodyToHeading(float x, float y, float z, float roll, float pitch, float *forward, float *left, float *up)
{
    const float sr = sinf(roll);
    const float cr = cosf(roll);
    const float sp = sinf(pitch);
    const float cp = cosf(pitch);
    *forward = cp * x + sp * sr * y + sp * cr * z;
    *left = cr * y - sr * z;
    *up = -sp * x + cp * sr * y + cp * cr * z;
}

// ZYX angle targets, degrees, that point the thrust along (forward, right, up) in the heading frame.
STATIC_UNIT_TESTED void bankAnglesFromThrust(float forward, float right, float up, float *rollDeg, float *pitchDeg)
{
    const float length = sqrtf(sq(forward) + sq(right) + sq(up));
    *rollDeg = RADIANS_TO_DEGREES(asinf(constrainf(right / fmaxf(length, 1e-6f), -1.0f, 1.0f)));
    *pitchDeg = RADIANS_TO_DEGREES(atan2f(forward, up));
}

// The transpose: a heading-frame vector in the body frame.
STATIC_UNIT_TESTED void bankHeadingToBody(float forward, float left, float up, float roll, float pitch, float *x, float *y, float *z)
{
    const float sr = sinf(roll);
    const float cr = cosf(roll);
    const float sp = sinf(pitch);
    const float cp = cosf(pitch);
    *x = cp * forward - sp * up;
    *y = sp * sr * forward + cr * left + cp * sr * up;
    *z = sp * cr * forward - sr * left + cp * cr * up;
}

// The velocity that makes a rotor drag velocity (x, y) in the rotor plane, assuming level travel: forward and left in
// the heading frame. Pitched and banked, part of the forward travel lands on body Y.
STATIC_UNIT_TESTED void bankLevelVelocity(float x, float y, float roll, float pitch, float *forward, float *left)
{
    *forward = x / fmaxf(cosf(pitch), BANK_LEVEL_MIN_COS);
    *left = (y - *forward * sinf(roll) * sinf(pitch)) / fmaxf(cosf(roll), BANK_LEVEL_MIN_COS);
}

// The Speed stick above min_check, 0..1, with a stop band at the bottom: the lower half is stretched over what is
// left, so the centre is still the centre.
STATIC_UNIT_TESTED float bankShapeSpeedStick(float stick)
{
    const float s = constrainf(stick, 0.0f, 1.0f);
    if (s >= 0.5f) {
        return s;
    }
    return 0.5f * fmaxf(s - BANK_STOP_STICK, 0.0f) / (0.5f - BANK_STOP_STICK);
}

// Speed target of the lower half of the Speed stick, g of drag: 0 at the stop to midSpeed at the centre, and below
// the stop (the reverse zone, a negative stick) backwards on the same line.
STATIC_UNIT_TESTED float bankTargetSpeed(float speedStick, float midSpeed)
{
    return 2.0f * constrainf(speedStick, -BANK_REVERSE_STICK, 0.5f) * midSpeed;
}

// Forward force, g, towards a speed target: the drag at that speed fed forward, plus P on the error.
STATIC_UNIT_TESTED float bankSpeedLoop(float targetSpeed, float v)
{
    return targetSpeed + BANK_SPEED_KP * (targetSpeed - v);
}

// Forward force, g, from the Speed stick (-BANK_REVERSE_STICK..1) and the speed v (g of drag).
// Lower half: the speed loop. Only the bottom third brakes a craft that is too fast; above it the force goes no lower
// than zero and the craft coasts down.
// Upper half: a straight line from the centre's force to maxForce, with no speed hold.
STATIC_UNIT_TESTED float bankSpeedForce(float speedStick, float v, float midSpeed, float maxForce)
{
    const float stick = constrainf(speedStick, -BANK_REVERSE_STICK, 1.0f);
    const float vTarget = bankTargetSpeed(stick, midSpeed);
    float fx = bankSpeedLoop(vTarget, v);
    if (stick > BANK_BRAKE_STICK) {
        fx = fmaxf(fx, 0.0f);
    }
    if (stick <= 0.5f) {
        return fx;
    }
    return fx + (2.0f * stick - 1.0f) * (fmaxf(maxForce, fx) - fx);
}

// Speed at which a sideways force (g) turns the track at turnRate (rad/s): omega = dragRate * force / speed.
STATIC_UNIT_TESTED float bankCornerSpeed(float force, float dragRate, float turnRate)
{
    return dragRate * force / fmaxf(turnRate, 0.1f);
}

// How far the corner speed has taken over the Speed stick, 0..1, for the Bank stick 0..1.
STATIC_UNIT_TESTED float bankCornerShare(float bankStick)
{
    return constrainf((bankStick - BANK_CORNER_START) / (BANK_CORNER_FULL - BANK_CORNER_START), 0.0f, 1.0f);
}

// The thrust vector in the travel frame (g), limited to thrust in priority order:
//   1. the corner force, the forward force (or brake) that holds the corner speed: without it a held full Bank bleeds
//      the speed until the turn is a spin, and with it a hard turn brakes before it banks
//   2. Bank, keeping its angle while Lift gives way, down to BANK_LIFT_YIELD of Lift, then the angle gives way
//   3. the forward force (or a brake), from what is left
// Brakes go no further back than BANK_MAX_BRAKE_DEG. bankAngle in radians, positive right.
STATIC_UNIT_TESTED void bankThrustVector(float lift, float thrust, float bankAngle, float cornerForce, float forwardForce, float *forward, float *right, float *up)
{
    const float brakeLimit = lift * tanf(DEGREES_TO_RADIANS(BANK_MAX_BRAKE_DEG));
    const float corner = constrainf(cornerForce, -brakeLimit, thrust);
    const float remaining = sqrtf(fmaxf(sq(thrust) - sq(corner), 0.0f));
    const float bank = constrainf(bankAngle, -DEGREES_TO_RADIANS(BANK_MAX_BANK_DEG), DEGREES_TO_RADIANS(BANK_MAX_BANK_DEG));
    float vertical = lift;
    float lateral = lift * tanf(fabsf(bank));
    if (sq(vertical) + sq(lateral) > sq(remaining)) {
        vertical = fminf(fmaxf(remaining * cosf(bank), BANK_LIFT_YIELD * lift), remaining);
        lateral = sqrtf(fmaxf(sq(remaining) - sq(vertical), 0.0f));
    }
    *up = vertical;
    *right = copysignf(lateral, bank);
    const float spare = sqrtf(fmaxf(sq(remaining) - sq(vertical) - sq(lateral), 0.0f));
    if (forwardForce >= 0.0f) {
        *forward = corner + fminf(forwardForce, spare);
    } else {
        *forward = corner - fminf(fminf(-forwardForce, spare), fmaxf(brakeLimit + corner, 0.0f));
    }
}

// Rate of the travel frame's heading, rad/s, counter-clockwise: the Euler yaw rate from the body rates.
STATIC_UNIT_TESTED float bankHeadingRate(float gyroPitch, float gyroYaw, float roll, float pitch)
{
    return (sinf(roll) * gyroPitch + cosf(roll) * gyroYaw) / fmaxf(cosf(pitch), BANK_LEVEL_MIN_COS);
}

// Fast thrust trim: measured thrust over the modelled one, dividing the thrust model. With 1 g or more up the craft
// may be resting on the ground, which hides missing thrust, so there it only moves back towards 1.
STATIC_UNIT_TESTED float bankUpdateThrustTrim(float trim, float measuredThrust, float modelThrust, bool mayBeOnGround, float dt)
{
    float target = measuredThrust / modelThrust;
    if (mayBeOnGround) {
        target = fmaxf(target, fminf(trim, 1.0f));
    }
    return trim + pt1FilterGain(BANK_TRIM_FILTER_HZ, dt) * (constrainf(target, BANK_TRIM_MIN, BANK_TRIM_MAX) - trim);
}

// On the ground any trim measures as right, and the motors spinning up walk it up, so a trim that takes thrust off
// leaks back to 1. One that adds thrust holds: fast flight needs it for as long as it lasts.
STATIC_UNIT_TESTED float bankLeakThrustTrim(float trim, float dt)
{
    return trim > 1.0f ? trim + dt / BANK_TRIM_LEAK_S * (1.0f - trim) : trim;
}

// Thrust lost in forward flight comes from the inflow through the props (speed * sin(forward tilt), g of drag). When
// the inflow drops (rolling into a bank, braking) the props get their thrust back faster than the trim can measure,
// so the thrust the trim adds is scaled down with the inflow at once.
STATIC_UNIT_TESTED float bankScaleTrimWithInflow(float trim, float inflow, float previousInflow)
{
    if (trim >= 1.0f || inflow >= previousInflow) {
        return trim;
    }
    const float share = (fmaxf(inflow, 0.0f) + BANK_TRIM_INFLOW_FLOOR) / (fmaxf(previousInflow, 0.0f) + BANK_TRIM_INFLOW_FLOOR);
    return 1.0f - (1.0f - trim) * share;
}

STATIC_UNIT_TESTED void bankCompute(const bankModeConfig_t *config, const bankInputs_t *in, bankState_t *state, bankOutputs_t *out)
{
    const bool restart = in->dt <= 0.0f || in->dt > BANK_MAX_DT_S;
    const float dt = restart ? BANK_FALLBACK_DT_S : in->dt;
    const float thrustToWeight = config->thrustToWeight / 100.0f;
    const float dragRate = config->dragRate / 100.0f;
    if (state->thrustTrim == 0.0f || !in->armed) {
        state->thrustTrim = 1.0f;
        state->budgetTrim = 1.0f;
    }

    const float pitch = DEGREES_TO_RADIANS(in->measuredPitchDeg);
    const float roll = DEGREES_TO_RADIANS(in->measuredRollDeg);
    // the measured thrust direction in the heading frame, a unit vector
    float tiltForward;
    float tiltLeft;
    float tiltCos;
    bankBodyToHeading(0.0f, 0.0f, 1.0f, roll, pitch, &tiltForward, &tiltLeft, &tiltCos);
    // the specific force in the heading frame
    float forceForward;
    float forceLeft;
    float forceUp;
    bankBodyToHeading(in->accX, in->accY, in->accZ, roll, pitch, &forceForward, &forceLeft, &forceUp);

    // the vertical part of the drag in the rotor plane, which the thrust needn't give: pitched far forward at speed the
    // rotor drag pushes up
    const float dragLiftSample = forceUp - in->accZ * tiltCos;
    const float liftGain = restart ? 1.0f : pt1FilterGain(BANK_DRAG_LIFT_HZ, dt);
    state->dragLift += liftGain * (dragLiftSample - state->dragLift);
    state->verticalForce += liftGain * (forceUp - state->verticalForce);

    // thrust trim, from the thrust the last collective was expected to give, measured along the thrust axis so it
    // holds at any tilt
    const float modelThrust = bankThrustFromCollective(state->collective, thrustToWeight);
    state->trimModelThrust = restart ? modelThrust
        : state->trimModelThrust + pt1FilterGain(BANK_TRIM_LAG_HZ, dt) * (modelThrust - state->trimModelThrust);
    // with the mixer saturated the thrust is not collective * T/W
    if (in->armed && !in->motorsSaturated && tiltCos > 0.0f && state->trimModelThrust >= BANK_TRIM_MIN_THRUST) {
        const bool mayBeOnGround = forceUp >= 1.0f && sq(state->velForward) + sq(state->velLeft) < sq(BANK_GROUND_SPEED);
        state->thrustTrim = bankUpdateThrustTrim(state->thrustTrim, in->accZ, state->trimModelThrust, mayBeOnGround, dt);
    }
    state->thrustTrim = bankLeakThrustTrim(state->thrustTrim, dt);
    const float trimmedThrustToWeight = thrustToWeight * state->thrustTrim;
    const float thrust = bankThrustFromCollective(state->collective, trimmedThrustToWeight);

    // velocity: the specific force integrated in the turning heading frame, corrected in the rotor plane by the rotor
    // drag, brought back to 1 g of thrust (rotor drag grows as sqrt(thrust)). Rotor drag pushes against the travel.
    const float dragScale = state->collective > 0.0f ? 1.0f / sqrtf(fmaxf(thrust, BANK_DRAG_MIN_THRUST)) : 1.0f;
    const float dragX = -in->accX * dragScale;
    const float dragY = -in->accY * dragScale;
    if (restart) {
        bankLevelVelocity(dragX, dragY, roll, pitch, &state->velForward, &state->velLeft);
        state->velUp = 0.0f;
    } else {
        const float headingStep = bankHeadingRate(DEGREES_TO_RADIANS(in->gyroPitchDps), DEGREES_TO_RADIANS(in->gyroYawDps), roll, pitch) * dt;
        bankRotate(&state->velForward, &state->velLeft, -headingStep);
        state->velForward += dragRate * forceForward * dt;
        state->velLeft += dragRate * forceLeft * dt;
        state->velUp += (dragRate * (forceUp - 1.0f) - BANK_VEL_UP_LEAK * state->velUp) * dt;
        float velX;
        float velY;
        float velZ;
        bankHeadingToBody(state->velForward, state->velLeft, state->velUp, roll, pitch, &velX, &velY, &velZ);
        const float dragGain = fminf(BANK_VEL_DRAG_GAIN * dt, 1.0f);
        float correctForward;
        float correctLeft;
        float correctUp;
        bankBodyToHeading(dragGain * (dragX - velX), dragGain * (dragY - velY), 0.0f, roll, pitch, &correctForward, &correctLeft, &correctUp);
        state->velForward += correctForward;
        state->velLeft += correctLeft;
        state->velUp += correctUp;
    }

    // speed along the track, negative travelling backwards, and the slip from the nose to the track's line
    const float velForward = state->velForward;
    const float velLeft = state->velLeft;
    const float travel = velForward >= -BANK_SPEED_DEADBAND ? 1.0f : -1.0f;
    const float speed = travel * fapplyDeadband(sqrtf(sq(velForward) + sq(velLeft)), BANK_SPEED_DEADBAND);
    const float turnShare = constrainf(fabsf(speed) / BANK_TURN_MIN_SPEED, 0.0f, 1.0f);
    const float slip = atan2f(travel * velLeft, fabsf(velForward)) * turnShare;

    const float inflow = fmaxf(speed, 0.0f) * fmaxf(tiltForward, 0.0f);
    state->thrustTrim = bankScaleTrimWithInflow(state->thrustTrim, inflow, state->trimInflow);
    state->trimInflow = inflow;

    float bankStick = bankShapeStick(in->bank);
    float speedStick = bankShapeSpeedStick(in->speed);
    if (state->tiltLocked) {
        if (bankStick == 0.0f && speedStick == 0.0f) {
            state->tiltLocked = false;
        } else {
            bankStick = 0.0f;
            speedStick = 0.0f;
        }
    }
    // the stick is at the bottom for arming, so no reverse until it has been raised
    if (!in->armed) {
        state->reverseReady = false;
    } else if (speedStick > 0.0f) {
        state->reverseReady = true;
    }
    if (state->reverseReady) {
        speedStick -= BANK_REVERSE_STICK * constrainf(in->reverse, 0.0f, 1.0f);
    }
    // the Lift stick springs back to the centre, so it gets the deadband there
    const float liftStick = 0.5f * (bankShapeStick(2.0f * in->lift - 1.0f) + 1.0f);

    // the thrust vector in the travel frame; at zero Speed there is no track to bank into, and the Bank stick yaws.
    // The thrust there is to share out follows the trim slowly: at the edge of the envelope the trim swings with the
    // inflow faster than the vector should
    state->budgetTrim += pt1FilterGain(BANK_BUDGET_HZ, dt) * (state->thrustTrim - state->budgetTrim);
    const float availableThrust = BANK_THRUST_SHARE * thrustToWeight * state->budgetTrim;
    const float lift = bankLiftFromStick(liftStick, thrustToWeight);
    // the vertical force loop: what the thrust model and the drag miss is integrated from the measured vertical force,
    // and asked of the thrust, so Lift holds even where the props lose most of their thrust; at a standstill the craft
    // may be on the ground, which reads as Lift
    const bool moving = sq(state->velForward) + sq(state->velLeft) >= sq(BANK_GROUND_SPEED);
    if (in->armed && liftStick > 0.0f && moving) {
        state->liftIntegral = constrainf(state->liftIntegral + BANK_LIFT_KI * (lift - state->verticalForce) * dt, -BANK_LIFT_I_MAX, BANK_LIFT_I_MAX);
    } else {
        state->liftIntegral -= state->liftIntegral * fminf(dt / BANK_LIFT_I_LEAK_S, 1.0f);
    }
    const float thrustLift = lift - state->dragLift + state->liftIntegral;
    const float liftForTilt = fmaxf(thrustLift, BANK_MIN_LIFT);
    const float bankAngle = bankStick * DEGREES_TO_RADIANS(BANK_MAX_BANK_DEG) * turnShare;
    const float midSpeed = config->midSpeed / 100.0f;
    const float horizontal = sqrtf(fmaxf(sq(availableThrust) - sq(liftForTilt), 0.0f));
    const float fullBankForce = sqrtf(fmaxf(sq(availableThrust) - sq(BANK_LIFT_YIELD * liftForTilt), 0.0f));
    const float cornerSpeed = bankCornerSpeed(fullBankForce, dragRate, DEGREES_TO_RADIANS(config->cornerRate));
    // near full Bank the forward force is shared, by the corner share, into the corner force towards the corner speed,
    // which comes before Bank
    const float cornerShare = bankCornerShare(fabsf(bankStick) * turnShare);
    const float forwardForce = bankSpeedForce(speedStick, speed, midSpeed, horizontal);
    const float cornerForce = cornerShare * fminf(bankSpeedLoop(cornerSpeed, speed), forwardForce);
    float forward;
    float right;
    float up;
    bankThrustVector(liftForTilt, availableThrust, bankAngle, cornerForce, (1.0f - cornerShare) * forwardForce, &forward, &right, &up);

    // into the heading frame: the slip turns the vector with the track, so a lagging nose doesn't move the force
    float left = -right;
    bankRotate(&forward, &left, slip);
    bankAnglesFromThrust(forward, -left, up, &out->rollDeg, &out->pitchDeg);

    // the collective holds the vertical force for the measured tilt, so Lift is right while the craft rotates, but
    // never more than there is. No thrust inverted.
    const float vertical = fminf(thrustLift, up);
    const float collective = tiltCos > 0.0f
        ? bankCollectiveFromThrust(fminf(fmaxf(vertical, 0.0f) / tiltCos, BANK_THRUST_SHARE * trimmedThrustToWeight), trimmedThrustToWeight)
        : 0.0f;
    state->collective = collective;

    // yaw: the track's turn rate fed forward from the sideways force the craft makes now, and the slip closed; nothing
    // turns with the Lift stick at the bottom (on the ground, or dropping, where the sink reads as slip)
    float turnRateDps = 0.0f;
    float yawRateDps = 0.0f;
    if (liftStick > 0.0f) {
        float travelForward = tiltForward;
        float travelLeft = tiltLeft;
        bankRotate(&travelForward, &travelLeft, -slip);
        turnRateDps = RADIANS_TO_DEGREES(dragRate * thrust * travelLeft * speed / fmaxf(sq(speed), sq(BANK_TURN_MIN_SPEED)));
        yawRateDps = turnRateDps + RADIANS_TO_DEGREES(BANK_SLIP_GAIN * slip)
            - bankStick * (1.0f - turnShare) * BANK_HOVER_YAW_RATE;
    }

    // the tilt: the measured thrust axis turned towards the vector, about the axis between them
    const float length = fmaxf(sqrtf(sq(forward) + sq(left) + sq(up)), 1e-6f);
    const float axisForward = (tiltLeft * up - tiltCos * left) / length;
    const float axisLeft = (tiltCos * forward - tiltForward * up) / length;
    const float axisUp = (tiltForward * left - tiltLeft * forward) / length;
    const float axisSin = sqrtf(sq(axisForward) + sq(axisLeft) + sq(axisUp));
    const float tiltError = atan2f(axisSin, (tiltForward * forward + tiltLeft * left + tiltCos * up) / length);
    const float tiltRate = fminf(BANK_TILT_GAIN * tiltError, DEGREES_TO_RADIANS(BANK_TILT_RATE)) / fmaxf(axisSin, 1e-6f);
    // and the heading's turn, about the world's up, both in the body frame
    out->yawRateDps = constrainf(yawRateDps, -BANK_YAW_MAX_RATE, BANK_YAW_MAX_RATE);
    const float turnRate = DEGREES_TO_RADIANS(out->yawRateDps);
    float rates[3];
    bankHeadingToBody(tiltRate * axisForward, tiltRate * axisLeft, tiltRate * axisUp + turnRate, roll, pitch, &rates[0], &rates[1], &rates[2]);
    for (int axis = 0; axis < 3; axis++) {
        out->rateDps[axis] = RADIANS_TO_DEGREES(rates[axis]);
    }
    out->collective = collective;
    out->throttleLow = liftStick <= 0.0f || collective <= 0.0f;

    DEBUG_SET(DEBUG_BANK, 0, lrintf(speed * 1000.0f));                                   //!< Speed Estimate [unit:0.001g]
    DEBUG_SET(DEBUG_BANK, 1, lrintf(cornerSpeed * 1000.0f));                             //!< Corner Speed [unit:0.001g]
    DEBUG_SET(DEBUG_BANK, 2, lrintf(state->thrustTrim * 1000.0f));                       //!< Thrust Trim [unit:0.001]
    DEBUG_SET(DEBUG_BANK, 3, lrintf(forward * 1000.0f));                                 //!< Forward Force [unit:0.001g]
    DEBUG_SET(DEBUG_BANK, 4, lrintf(right * 1000.0f));                                   //!< Sideways Force [unit:0.001g]
    DEBUG_SET(DEBUG_BANK, 5, lrintf(RADIANS_TO_DEGREES(slip) * 10.0f));                  //!< Slip [unit:0.1deg]
    DEBUG_SET(DEBUG_BANK, 6, lrintf(turnRateDps * 10.0f));                               //!< Turn Feedforward [unit:0.1dps]
    DEBUG_SET(DEBUG_BANK, 7, lrintf((lift - 1.0f) * 1000.0f));                           //!< Vertical Force Command [unit:0.001g]
}

void bankModeUpdate(timeUs_t currentTimeUs)
{
    if (!FLIGHT_MODE(BANK_MODE)) {
        bankState = (bankState_t){ 0 };
        bankOutputs = (bankOutputs_t){ 0 };
        return;
    }

    float dt = 0.0f;
    if (bankState.lastUpdateUs == 0) {
        // engaged with the throttle low (arming): no tilt until the Bank stick has been centred once
        bankState.tiltLocked = calculateThrottleStatus() == THROTTLE_LOW;
    } else {
        dt = cmpTimeUs(currentTimeUs, bankState.lastUpdateUs) * 1e-6f;
        if (dt < BANK_MIN_DT_S) {
            return;
        }
    }
    bankState.lastUpdateUs = currentTimeUs;

    const bankInputs_t inputs = {
        // the yaw deflection is positive to the left
        .bank = -getRcDeflection(FD_YAW),
        .speed = (rcCommand[THROTTLE] - PWM_RANGE_MIN) / (float)PWM_RANGE,
        // rcCommand is flat below min_check, so the reverse zone reads the raw stick
        .reverse = (rxConfig()->mincheck - rcData[THROTTLE]) / (float)MAX(rxConfig()->mincheck - PWM_RANGE_MIN, 1),
        .lift = 0.5f * (getRcDeflection(FD_PITCH) + 1.0f),
        .armed = ARMING_FLAG(ARMED),
        .accX = acc.accADC.x * acc.dev.acc_1G_rec,
        .accY = acc.accADC.y * acc.dev.acc_1G_rec,
        .accZ = acc.accADC.z * acc.dev.acc_1G_rec,
        .gyroPitchDps = gyro.gyroADCf[FD_PITCH],
        .gyroYawDps = gyro.gyroADCf[FD_YAW],
        .measuredPitchDeg = attitude.raw[FD_PITCH] * 0.1f,
        .measuredRollDeg = attitude.raw[FD_ROLL] * 0.1f,
        .dt = dt,
        .motorsSaturated = getMotorMixRange() >= 1.0f,
    };
    bankCompute(bankModeConfig(), &inputs, &bankState, &bankOutputs);
}

bool bankModeIsActive(void)
{
    return FLIGHT_MODE(BANK_MODE);
}

float bankModeGetAngleTarget(int axis)
{
    return axis == FD_ROLL ? bankOutputs.rollDeg : bankOutputs.pitchDeg;
}

float bankModeGetRate(int axis)
{
    return bankOutputs.rateDps[axis];
}

float bankModeGetCollective(void)
{
    return bankOutputs.collective;
}

float bankModeGetThrustToWeight(void)
{
    return bankModeConfig()->thrustToWeight / 100.0f;
}

// The bottom of the throttle stick is the Speed stick's stop, not idle, so the throttle is low with the Lift stick at
// the bottom (its floor still gives thrust) or no collective.
bool bankModeIsThrottleLow(void)
{
    return bankOutputs.throttleLow;
}

#endif // USE_BANK_MODE
