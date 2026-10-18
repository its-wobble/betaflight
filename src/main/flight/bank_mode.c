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

#include "bank_mode.h"

/*
 * BANK mode: fly a multirotor like a car on a banked track.
 *
 * Attitude only (gyro/ACC), no GPS or baro. Forces are in g (weight = 1). Roll is positive to the right, pitch
 * positive nose-down, body axes are X forward, Y left, Z up, and yaw is positive counter-clockwise.
 *
 * Sticks (mode 2):
 *   throttle  Speed: the lower half is a speed target, 0 to bank_mid_speed, with a stop band at the bottom; the upper
 *             half a forward force up to what the thrust allows. Below min_check a small reverse, once the stick has
 *             been raised since arming.
 *   pitch     Lift: 0 g at the bottom, 1 g at the centre, all the thrust at the top
 *   yaw       Bank: bank angle, the turn follows from it
 *   roll      head check: nose offset from the direction of travel, springs back at the centre
 *
 * The sticks set a thrust vector in the travel frame (x forward along the track, r to the right, z up):
 *   z = Lift,  r = Lift * tan(bank),  x = the forward force for the Speed stick
 * Thrust priority is Lift, then Bank, then Speed, but a brake comes before Bank: the horizontal part (x, r) gets at
 * most H = sqrt(Tb^2 - Lift^2), Tb leaving a quarter of the spare thrust to the attitude loop, and a braking x is
 * taken out of it first, then r, then a forward x. The vector is rotated
 * by the nose offset (head check and look into the turn), so the nose moves but the track doesn't, and becomes
 *   pitch = asin(x / |thrust|),  roll = atan2(r, z),  collective from Lift / cos(measured tilt)
 *
 * Speed is the levelled accelerometer drag normalised by sqrt(thrust), in g of drag, so it needs no m/s calibration.
 * Yaw feeds forward the coordinated turn rate g * tan(bank) / v and keeps the ball (the lateral drag in the travel
 * frame) centred, with an integral in turns for what the feedforward misses. A fast thrust trim corrects the modelled
 * thrust from the measured vertical force.
 *
 * Limitations:
 *   - the ball and the speed only see rotor drag: little signal at low speed, and accelerometer bias reads as speed
 *   - a climb or sink with the craft tilted puts air across the rotors, which reads as speed or slip
 *   - Lift is a force: nothing holds the height or the vertical speed
 *
 * OPEN: interaction with HEADFREE and arming; coexistence with ALT_HOLD/POS_HOLD (they currently cancel BANK).
 */

#define BANK_MIN_DT_S           0.0005f
#define BANK_MAX_DT_S           0.2f    // longer gaps restart the filters
#define BANK_FALLBACK_DT_S      0.01f
#define BANK_STICK_DEADBAND     0.05f
#define BANK_THRUST_EXPONENT    1.5f    // thrust ~ collective ^ 1.5
#define BANK_TILT_THRUST_SHARE  0.75f   // share of the spare thrust the tilt may use
#define BANK_MIN_LIFT           0.5f    // g, the lowest lift the tilt is computed for
#define BANK_MAX_BRAKE_DEG      30.0f
#define BANK_BRAKE_STICK        (1.0f / 3.0f)   // below this Speed stick a craft that is too fast is braked
#define BANK_REVERSE_STICK      0.05f   // the reverse zone carries the Speed stick this far below 0
#define BANK_STOP_STICK         0.10f   // Speed stick above min_check that still asks for no speed
#define BANK_SPEED_KP           2.0f
#define BANK_SPEED_DEADBAND     0.02f   // g of drag
#define BANK_DRAG_FILTER_HZ     2.0f
#define BANK_DRAG_MIN_THRUST    0.25f   // g
#define BANK_LEVEL_MIN_COS      0.087f  // cos(85 deg), the angle loop's limit
#define BANK_YAW_MAX_RATE       720.0f  // deg/s; in a steep bank most of it is pitch rate
#define BANK_NOSE_SLEW_RATE     360.0f  // deg/s, how fast the nose offset (head check, look) moves
#define BANK_TURN_GAIN          19.0f   // deg/s * g of drag, g in the turn rate g * tan(bank) / v
#define BANK_TURN_MIN_SPEED     0.1f    // g of drag, the turn feedforward fades out below this
#define BANK_TURN_RATE_SHARE    0.6f    // share of the yaw limit the feedforward may use, the rest is the ball's
#define BANK_TURN_MAX_BANK_DEG  85.0f
#define BANK_BALL_KP            600.0f  // deg/s per g
#define BANK_BALL_KI            600.0f  // deg/s per g per s
#define BANK_TRIM_MIN_TILT_COS  0.25f   // no trim measurement past 75 deg of tilt
#define BANK_TRIM_MIN_LIFT      0.3f    // g, no trim measurement below this modelled vertical force
#define BANK_TRIM_FILTER_HZ     3.0f
#define BANK_TRIM_LAG_HZ        10.0f   // motor and accelerometer lag, applied to the modelled thrust
#define BANK_TRIM_MIN           0.2f
#define BANK_TRIM_MAX           2.0f
#define BANK_TRIM_LEAK_S        1.0f
#define BANK_TRIM_INFLOW_FLOOR  0.1f    // g of drag
#define BANK_LOOK_FADE_STICK    0.2f    // Bank stick travel over which the look into the turn fades in

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

// Lift in g for the Lift stick (0..1): linear from 0 to 1 g below the centre, then 1 + x + (T/W - 2) * x^3 up to T/W,
// so both halves start at 1 g per half stick and the centre isn't touchy.
STATIC_UNIT_TESTED float bankLiftFromStick(float stick, float thrustToWeight)
{
    const float s = constrainf(stick, 0.0f, 1.0f);
    if (s <= 0.5f) {
        return 2.0f * s;
    }
    const float x = 2.0f * s - 1.0f;
    const float tw = fmaxf(thrustToWeight, 1.0f);
    if (tw <= 2.0f) {
        return 1.0f + x * (tw - 1.0f);     // no room for the curve
    }
    return 1.0f + x + (tw - 2.0f) * x * x * x;
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

// Rotates a horizontal vector (forward, right) counter-clockwise by angle: from the travel frame into the nose frame
// when angle is the nose offset, and back with -angle.
STATIC_UNIT_TESTED void bankRotate(float *forward, float *right, float angle)
{
    const float c = cosf(angle);
    const float s = sinf(angle);
    const float x = *forward;
    *forward = x * c - *right * s;
    *right = x * s + *right * c;
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

// Forward force, g, from the Speed stick (-BANK_REVERSE_STICK..1) and the speed v (g of drag).
// Lower half: feedforward plus P towards the target speed. Only the bottom third brakes a craft that is too fast;
// above it the P term only adds and the craft coasts down to the target.
// Upper half: a straight line from the centre's force to maxForce, with no speed hold.
STATIC_UNIT_TESTED float bankSpeedForce(float speedStick, float v, float midSpeed, float maxForce)
{
    const float stick = constrainf(speedStick, -BANK_REVERSE_STICK, 1.0f);
    const float vTarget = bankTargetSpeed(stick, midSpeed);
    float error = vTarget - v;
    if (stick > BANK_BRAKE_STICK) {
        error = fmaxf(error, 0.0f);
    }
    const float fx = vTarget + BANK_SPEED_KP * error;
    if (stick <= 0.5f) {
        return fx;
    }
    return fx + (2.0f * stick - 1.0f) * (fmaxf(maxForce, fx) - fx);
}

// Thrust the tilt may use at lift (g): the lift plus BANK_TILT_THRUST_SHARE of the spare thrust, so the mixer keeps
// headroom for the attitude loop in a hard bank.
STATIC_UNIT_TESTED float bankTiltThrust(float lift, float thrustToWeight)
{
    return lift + BANK_TILT_THRUST_SHARE * fmaxf(thrustToWeight - lift, 0.0f);
}

// The horizontal part of the thrust vector in the travel frame, limited in priority order: a brake (a negative forward
// force, at most a BANK_MAX_BRAKE_DEG tilt back) is taken out of the horizontal force budget first, Bank takes what it
// needs of the rest, and a forward force gets what is left. bankStick -1..1 is the bank angle as a share of 90 degrees.
STATIC_UNIT_TESTED void bankThrustVector(float lift, float horizontal, float bankStick, float forwardForce, float *forward, float *right)
{
    const float brakeMax = fminf(lift * tanf(DEGREES_TO_RADIANS(BANK_MAX_BRAKE_DEG)), horizontal);
    const float brake = fminf(fmaxf(-forwardForce, 0.0f), brakeMax);
    const float bankMax = atan2f(sqrtf(fmaxf(sq(horizontal) - sq(brake), 0.0f)), lift);
    const float bank = fminf(fabsf(bankStick) * M_PIf / 2.0f, bankMax);
    *right = copysignf(lift * tanf(bank), bankStick);
    const float forwardMax = sqrtf(fmaxf(sq(horizontal) - sq(*right), 0.0f));
    *forward = forwardForce < 0.0f ? -brake : fminf(forwardForce, forwardMax);
}

// Levels the body-frame drag with the measured attitude, assuming horizontal travel: forward and left along the nose's
// heading. Pitched and banked, part of the forward drag lands on body Y and would otherwise read as slip.
STATIC_UNIT_TESTED void bankLevelDrag(float accX, float accY, float pitch, float roll, float *forward, float *left)
{
    *forward = accX / fmaxf(cosf(pitch), BANK_LEVEL_MIN_COS);
    *left = (accY - *forward * sinf(roll) * sinf(pitch)) / fmaxf(cosf(roll), BANK_LEVEL_MIN_COS);
}

// +1 forwards, -1 clearly backwards, with a ramp between, so travel straight sideways doesn't flip the ball's sense.
// forwardDrag is the travel-frame drag, positive when travelling backwards.
STATIC_UNIT_TESTED float bankTravelDirection(float forwardDrag)
{
    return 1.0f - 2.0f * constrainf((forwardDrag - BANK_SPEED_DEADBAND) / BANK_SPEED_DEADBAND, 0.0f, 1.0f);
}

// Yaw rate of a coordinated turn, deg/s, positive counter-clockwise: g * tan(bank) / v, which with the speed in g of
// drag is BANK_TURN_GAIN * tan(bank) / speed. Fades out below BANK_TURN_MIN_SPEED. Reversing turns the other way.
STATIC_UNIT_TESTED float bankTurnFeedforward(float bankTan, float speed)
{
    const float rate = -BANK_TURN_GAIN * bankTan * speed / fmaxf(sq(speed), sq(BANK_TURN_MIN_SPEED));
    const float maxRate = BANK_TURN_RATE_SHARE * BANK_YAW_MAX_RATE;
    return constrainf(rate, -maxRate, maxRate);
}

// Fast thrust trim: measured vertical force over the modelled one, dividing the thrust model. It takes off excess
// thrust (braking into the air, rolling into a bank) and adds back what fast forward flight loses. At 1 g or more the
// craft may be resting on the ground, which hides missing thrust, so there it only moves back towards 1.
STATIC_UNIT_TESTED float bankUpdateThrustTrim(float trim, float verticalForce, float modelVertical, float dt)
{
    float target = verticalForce / modelVertical;
    if (verticalForce >= 1.0f) {
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

// Nose offset that looks into the turn: atan2(Bank, Speed), faded in over the first BANK_LOOK_FADE_STICK of the Bank
// stick. bankStick -1..1 (positive right), speedStick 0..1. Returns radians, positive counter-clockwise.
STATIC_UNIT_TESTED float bankLookIntoTurnOffset(float bankStick, float speedStick)
{
    const float bank = fabsf(bankStick);
    const float look = atan2f(bank, fmaxf(speedStick, 0.0f)) * fminf(bank / BANK_LOOK_FADE_STICK, 1.0f);
    return bankStick > 0.0f ? -look : look;
}

STATIC_UNIT_TESTED void bankCompute(const bankModeConfig_t *config, const bankInputs_t *in, bankState_t *state, bankOutputs_t *out)
{
    const bool restart = in->dt <= 0.0f || in->dt > BANK_MAX_DT_S;
    const float dt = restart ? BANK_FALLBACK_DT_S : in->dt;
    const float thrustToWeight = config->thrustToWeight / 100.0f;
    if (state->thrustTrim == 0.0f || !in->armed) {
        state->thrustTrim = 1.0f;
    }

    // the measured thrust direction in the nose frame, a unit vector
    const float pitch = DEGREES_TO_RADIANS(in->measuredPitchDeg);
    const float roll = DEGREES_TO_RADIANS(in->measuredRollDeg);
    const float tiltForward = sinf(pitch);
    const float tiltRight = cosf(pitch) * sinf(roll);
    const float tiltCos = cosf(pitch) * cosf(roll);

    // thrust trim, from the vertical force the last collective was expected to give
    const float modelVertical = bankThrustFromCollective(state->collective, thrustToWeight) * tiltCos;
    state->trimModelVertical = restart ? modelVertical
        : state->trimModelVertical + pt1FilterGain(BANK_TRIM_LAG_HZ, dt) * (modelVertical - state->trimModelVertical);
    // with the mixer saturated the thrust is not collective * T/W
    if (in->armed && !in->motorsSaturated && tiltCos >= BANK_TRIM_MIN_TILT_COS && state->trimModelVertical >= BANK_TRIM_MIN_LIFT) {
        const float verticalForce = -tiltForward * in->accX + tiltRight * in->accY + tiltCos * in->accZ;
        state->thrustTrim = bankUpdateThrustTrim(state->thrustTrim, verticalForce, state->trimModelVertical, dt);
    }
    state->thrustTrim = bankLeakThrustTrim(state->thrustTrim, dt);

    // drag, levelled per sample and brought back to 1 g of thrust (rotor drag grows as sqrt(thrust))
    const float thrust = bankThrustFromCollective(state->collective, thrustToWeight * state->thrustTrim);
    const float dragScale = state->collective > 0.0f ? 1.0f / sqrtf(fmaxf(thrust, BANK_DRAG_MIN_THRUST)) : 1.0f;
    float levelX;
    float levelY;
    bankLevelDrag(in->accX * dragScale, in->accY * dragScale, pitch, roll, &levelX, &levelY);
    // filtered in the travel frame, so the nose turning away from the track (head check, look) isn't read as slip
    // through the filter's lag
    float sampleForward = levelX;
    float sampleRight = -levelY;
    bankRotate(&sampleForward, &sampleRight, -state->noseOffset);
    const float dragGain = restart ? 1.0f : pt1FilterGain(BANK_DRAG_FILTER_HZ, dt);
    state->dragForward += dragGain * (sampleForward - state->dragForward);
    state->dragRight += dragGain * (sampleRight - state->dragRight);

    // speed and ball in the travel frame
    const float dragForward = state->dragForward;
    const float dragRight = state->dragRight;
    const float direction = bankTravelDirection(dragForward);
    const float speed = direction * fapplyDeadband(sqrtf(sq(dragForward) + sq(dragRight)), BANK_SPEED_DEADBAND);
    const float ball = direction * dragRight;

    // the measured tilt in the travel frame
    float heldForward = tiltForward;
    float heldRight = tiltRight;
    bankRotate(&heldForward, &heldRight, -state->noseOffset);

    const float inflow = fmaxf(speed, 0.0f) * fmaxf(heldForward, 0.0f);
    state->thrustTrim = bankScaleTrimWithInflow(state->thrustTrim, inflow, state->trimInflow);
    state->trimInflow = inflow;

    float bankStick = bankShapeStick(in->roll);
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
    const float headStick = bankShapeStick(in->yaw);
    // the Lift stick springs back to the centre, so it gets the deadband there
    const float liftStick = 0.5f * (bankShapeStick(2.0f * in->lift - 1.0f) + 1.0f);

    // the props make less thrust in fast flight, so the limits use the trimmed thrust
    const float trimmedThrustToWeight = thrustToWeight * state->thrustTrim;
    const float lift = fminf(bankLiftFromStick(liftStick, thrustToWeight), trimmedThrustToWeight);
    const float liftForTilt = fmaxf(lift, BANK_MIN_LIFT);
    const float tiltThrust = bankTiltThrust(liftForTilt, trimmedThrustToWeight);
    const float horizontal = sqrtf(sq(tiltThrust) - sq(liftForTilt));
    const float midSpeed = config->midSpeed / 100.0f;
    const float forwardForce = bankSpeedForce(speedStick, speed, midSpeed, horizontal);
    float forward;
    float right;
    bankThrustVector(liftForTilt, horizontal, bankStick, forwardForce, &forward, &right);

    // the collective is set for the measured tilt, so Lift is right while the craft rotates to the target, but never
    // for more than the tilt budget, so an overshoot can't take it to full. No thrust inverted.
    const float collective = tiltCos > 0.0f
        ? bankCollectiveFromThrust(lift / fmaxf(tiltCos, liftForTilt / tiltThrust), trimmedThrustToWeight)
        : 0.0f;
    state->collective = collective;

    // nose offset: the look into the turn plus the head check, slewed
    const float offsetTarget = bankLookIntoTurnOffset(bankStick, speedStick) * (config->lookTurn / 100.0f)
        + headStick * DEGREES_TO_RADIANS(config->headCheckRange);
    const float maxStep = DEGREES_TO_RADIANS(BANK_NOSE_SLEW_RATE) * dt;
    const float offsetStep = constrainf(offsetTarget - state->noseOffset, -maxStep, maxStep);
    state->noseOffset += offsetStep;

    bankRotate(&forward, &right, state->noseOffset);
    const float liftVector = sqrtf(sq(forward) + sq(right) + sq(liftForTilt));

    // the turn rate is fed forward from the bank the craft is in, so a reversal doesn't turn the nose before the craft
    // has rolled through; nothing turns with no lift (on the ground or falling). The ball fades out with the speed like
    // the feedforward: with no track to follow an accelerometer bias would yaw the craft for good.
    const float ballYaw = ball * constrainf(fabsf(speed) / BANK_TURN_MIN_SPEED, 0.0f, 1.0f);
    float turnRateDps = 0.0f;
    float ballRateDps = 0.0f;
    if (lift > 0.0f) {
        const float maxBankTan = tanf(DEGREES_TO_RADIANS(BANK_TURN_MAX_BANK_DEG));
        const float heldBankTan = constrainf(heldRight / fmaxf(tiltCos, 1e-3f), -maxBankTan, maxBankTan);
        turnRateDps = bankTurnFeedforward(heldBankTan, speed);
        ballRateDps = BANK_BALL_KP * ballYaw;
    }
    const float yawRateDps = turnRateDps + ballRateDps + state->ballIntegral + RADIANS_TO_DEGREES(offsetStep / dt);
    // the integral takes out what the feedforward's fixed turn gain misses, so the nose holds the track; only in a
    // turn, and not deeper into the yaw limit
    if (bankStick == 0.0f || lift <= 0.0f) {
        state->ballIntegral = 0.0f;
    } else if (fabsf(yawRateDps) < BANK_YAW_MAX_RATE || ballYaw * yawRateDps < 0.0f) {
        state->ballIntegral += BANK_BALL_KI * ballYaw * dt;
    }

    out->pitchDeg = RADIANS_TO_DEGREES(asinf(forward / liftVector));
    out->rollDeg = RADIANS_TO_DEGREES(atan2f(right, liftForTilt));
    out->yawRateDps = constrainf(yawRateDps, -BANK_YAW_MAX_RATE, BANK_YAW_MAX_RATE);
    out->collective = collective;

    DEBUG_SET(DEBUG_BANK, 0, lrintf(speed * 1000.0f));                                   //!< Speed Estimate [unit:0.001g]
    DEBUG_SET(DEBUG_BANK, 1, lrintf(bankTargetSpeed(speedStick, midSpeed) * 1000.0f));   //!< Target Speed [unit:0.001g]
    DEBUG_SET(DEBUG_BANK, 2, lrintf(state->thrustTrim * 1000.0f));                       //!< Thrust Trim [unit:0.001]
    DEBUG_SET(DEBUG_BANK, 3, lrintf(forwardForce * 1000.0f));                            //!< Forward Force Command [unit:0.001g]
    DEBUG_SET(DEBUG_BANK, 4, lrintf(thrustToWeight * 100.0f));                           //!< Thrust To Weight [unit:0.01]
    DEBUG_SET(DEBUG_BANK, 5, lrintf(turnRateDps * 10.0f));                               //!< Turn Feedforward [unit:0.1dps]
    DEBUG_SET(DEBUG_BANK, 6, lrintf(ball * 1000.0f));                                    //!< Ball [unit:0.001g]
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
        // Bank on the yaw stick, the head check on the roll stick; the yaw deflection is positive to the left
        .roll = -getRcDeflection(FD_YAW),
        .speed = (rcCommand[THROTTLE] - PWM_RANGE_MIN) / (float)PWM_RANGE,
        // rcCommand is flat below min_check, so the reverse zone reads the raw stick
        .reverse = (rxConfig()->mincheck - rcData[THROTTLE]) / (float)MAX(rxConfig()->mincheck - PWM_RANGE_MIN, 1),
        .yaw = -getRcDeflection(FD_ROLL),
        .lift = 0.5f * (getRcDeflection(FD_PITCH) + 1.0f),
        .armed = ARMING_FLAG(ARMED),
        .accX = acc.accADC.x * acc.dev.acc_1G_rec,
        .accY = acc.accADC.y * acc.dev.acc_1G_rec,
        .accZ = acc.accADC.z * acc.dev.acc_1G_rec,
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

float bankModeGetYawRate(void)
{
    return bankOutputs.yawRateDps;
}

float bankModeGetCollective(void)
{
    return bankOutputs.collective;
}

float bankModeGetThrustToWeight(void)
{
    return bankModeConfig()->thrustToWeight / 100.0f;
}

// The bottom of the throttle stick is the Speed stick's stop, not idle, so the throttle is low only with no collective.
bool bankModeIsThrottleLow(void)
{
    return bankOutputs.collective <= 0.0f;
}

#endif // USE_BANK_MODE
