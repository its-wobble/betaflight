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

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "pg/bank_mode.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"

#include "bank_mode.h"

/*
 * BANK mode: fly a multirotor like a car on a banked track.
 *
 * Attitude only (gyro/ACC): no GPS, baro or velocity estimate is needed.
 * Forces are in multiples of the craft's weight (m*g = 1), so the thrust limit is Tmax = T/W.
 * Angles: roll is positive to the right, pitch is positive nose-down (forward), as in attitude.raw[].
 * Body axes are X forward, Y left, Z up, and yaw rates are positive counter-clockwise.
 *
 * Stick mapping (mode 2):
 *   throttle  -> vertical lift command (not raw throttle). The bottom of the stick gives no lift at all (free
 *                fall, the safe pose for arming), the centre gives 1 g (thrust balances weight), and the top gives
 *                the maximum thrust, all of it as lift, whatever multiple of the weight that is.
 *   yaw       -> head check: nose offset from the travel direction, springs back to 0 at centre.
 *   pitch     -> forward/backward force command, realised as tilt theta plus collective. Centre = none.
 *   roll      -> bank angle phi, i.e. a turn in the horizontal plane. Centre = none.
 *
 * Arming is therefore with the left stick at the bottom (no lift) and the right stick centred (no tilt).
 * If the mode is engaged with the throttle low and the right stick off-centre, the tilt stays at zero until
 * the stick has been centred once.
 *
 * Each update, while BANK_MODE is active:
 *
 *   phi   = rollStick * maxBankAngle
 *   theta = atan(pitchStick * tan(maxTiltAngle))        // forward force Fx = (1 + Fz) * tan(theta)
 *   Fz    = lift(throttleStick) - 1        // lift: 0 at the bottom, 1 at the centre, Tmax at the top, linear between
 *
 *   Thrust saturation, in priority order: Fz first, then bank (phi), forward force (theta) last.
 *     Fz   = min(Fz, Tmax - 1)
 *     cosTiltMin = (1 + Fz) / Tmax                      // T <= Tmax  <=>  cos(theta) * cos(phi) >= cosTiltMin
 *     phi   = min(phi, acos(cosTiltMin))                // bank takes what it needs first
 *     theta = min(theta, acos(cosTiltMin / cos(phi)))   // forward tilt gets what remains
 *
 *   The collective is always computed from the limited tilt, never taken from the stick:
 *     T = (1 + Fz) / (cos(theta) * cos(phi)),  collective = T / Tmax
 *
 *   Head check: the offset slews towards yawStick * headCheckRange. The thrust direction is built
 *   in the travel frame and rotated into the body frame by the offset, so the earth-frame thrust
 *   direction, and so the travel direction, is not changed by looking away. Tilt magnitude and
 *   collective are invariant under that rotation.
 *
 *   The nose follows the direction of travel without a velocity estimate. The accelerometer measures
 *   thrust (along body Z) plus drag, so its X/Y reading is drag, which points opposite to the
 *   velocity. The angle beta between the nose and the travel direction comes from that, and
 *     yawRate = gain * fade * (beta + headCheckOffset) + headCheckSlewRate
 *   where fade removes the feedback at low drag, where the direction is meaningless. This turns a
 *   bank into a turn like a car on a banked road: the bank tilts the thrust sideways, the craft
 *   moves that way, the nose follows the velocity, and the thrust stays beside the nose.
 *   When moving backwards the tail is aligned with the travel direction instead, like a reversing car.
 *
 * Limitations:
 *   - A steady turn leaves a small slip angle (turn rate / gain).
 *   - The travel direction relies on ACC calibration: trim errors look like drag.
 *   - Below the drag threshold the head check is open-loop (slew rate feed-forward only).
 *   - Tmax is not filtered, so it can step if the battery voltage drops.
 *
 * OPEN: interaction with HEADFREE and arming; coexistence with ALT_HOLD/POS_HOLD (they currently cancel BANK).
 */

#define BANK_DRAG_FILTER_HZ     2.0f    // cutoff of the accelerometer drag filter
#define BANK_MIN_DT_S           0.0005f
#define BANK_MAX_DT_S           0.2f    // longer gaps restart the filters
#define BANK_FALLBACK_DT_S      0.01f
#define BANK_MIN_TILT_COS       1e-3f

static bankState_t bankState;

static FAST_DATA_ZERO_INIT float bankAngleTargetDeg[RP_AXIS_COUNT];
static FAST_DATA_ZERO_INIT float bankYawRateDps;
static FAST_DATA_ZERO_INIT float bankCollective;
static FAST_DATA_ZERO_INIT bool bankModeActive;

// Removes the deadband and rescales so full deflection still gives 1.
STATIC_UNIT_TESTED float bankShapeStick(float deflection, float deadband)
{
    const float d = constrainf(deflection, -1.0f, 1.0f);
    const float magnitude = fabsf(d);
    if (magnitude <= deadband) {
        return 0.0f;
    }
    const float shaped = (magnitude - deadband) / (1.0f - deadband);
    return d < 0.0f ? -shaped : shaped;
}

// Vertical lift, in multiples of the weight, for the throttle stick position (0 bottom, 1 top):
// none at the bottom, 1 g at the centre, and all of the maximum thrust at the top.
STATIC_UNIT_TESTED float bankLiftFromStick(float stick, float thrustToWeight)
{
    const float s = constrainf(stick, 0.0f, 1.0f);
    if (s <= 0.5f) {
        return 2.0f * s;
    }
    return 1.0f + (2.0f * s - 1.0f) * (fmaxf(thrustToWeight, 1.0f) - 1.0f);
}

// Limits the tilt so the collective needed for the vertical force stays within thrust-to-weight.
// Fz has priority, then bank (phi), then forward tilt (theta). Returns the (possibly capped) Fz.
STATIC_UNIT_TESTED float bankLimitTilt(float fz, float *theta, float *phi, float thrustToWeight)
{
    const float tw = fmaxf(thrustToWeight, 1.0f);
    fz = constrainf(fz, -1.0f, tw - 1.0f);

    const float cosTiltMin = (1.0f + fz) / tw;

    const float phiMax = acosf(constrainf(cosTiltMin, 0.0f, 1.0f));
    *phi = constrainf(*phi, -phiMax, phiMax);

    const float cosPhi = fmaxf(cosf(*phi), BANK_MIN_TILT_COS);
    const float thetaMax = acosf(constrainf(cosTiltMin / cosPhi, 0.0f, 1.0f));
    *theta = constrainf(*theta, -thetaMax, thetaMax);

    return fz;
}

// Collective as a fraction of the maximum thrust, from the vertical force and the tilt.
STATIC_UNIT_TESTED float bankCalcCollective(float fz, float theta, float phi, float thrustToWeight)
{
    const float cosTilt = fmaxf(cosf(theta) * cosf(phi), BANK_MIN_TILT_COS);
    const float thrust = (1.0f + fz) / cosTilt;
    return constrainf(thrust / fmaxf(thrustToWeight, 1.0f), 0.0f, 1.0f);
}

// Moves the head-check offset towards the target by at most rate * dt. Returns the new offset.
STATIC_UNIT_TESTED float bankSlewHeadCheck(float offset, float target, float rate, float dt)
{
    const float maxStep = rate * dt;
    return offset + constrainf(target - offset, -maxStep, maxStep);
}

// Expresses a tilt (theta, phi) given in the travel frame in the body frame, when the nose is
// rotated counter-clockwise by offset from the travel direction. Uses the body-frame vertical
// u = (-sin(theta), cos(theta) * sin(phi), cos(theta) * cos(phi)), the same quantity that
// attitude.raw[] roll and pitch are taken from. The horizontal part is rotated, u.z is unchanged.
STATIC_UNIT_TESTED void bankRotateTilt(float *theta, float *phi, float offset)
{
    const float cosTheta = cosf(*theta);
    const float ux = -sinf(*theta);
    const float uy = cosTheta * sinf(*phi);
    const float uz = cosTheta * cosf(*phi);

    const float c = cosf(offset);
    const float s = sinf(offset);
    const float bx = ux * c + uy * s;
    const float by = -ux * s + uy * c;

    *theta = asinf(constrainf(-bx, -1.0f, 1.0f));
    *phi = atan2f(by, uz);
}

// Yaw rate (degrees per second, counter-clockwise positive) that turns the nose towards the
// direction of travel, from the drag the accelerometer sees. dragX/dragY are specific forces in g.
STATIC_UNIT_TESTED float bankCalcAlignYawRate(float dragX, float dragY, float offset, float gainPerSecond, float dragThreshold)
{
    const float magnitude = sqrtf(dragX * dragX + dragY * dragY);
    const float fadeStart = dragThreshold;
    const float fadeEnd = 2.0f * dragThreshold;
    float fade;
    if (magnitude <= fadeStart) {
        fade = 0.0f;
    } else if (magnitude >= fadeEnd) {
        fade = 1.0f;
    } else {
        const float x = (magnitude - fadeStart) / (fadeEnd - fadeStart);
        fade = x * x * (3.0f - 2.0f * x);
    }
    if (fade <= 0.0f) {
        return 0.0f;
    }

    // drag opposes the velocity, so the travel direction in the body frame is -drag
    float beta = atan2f(-dragY, -dragX);
    // reversing: align the tail with the travel direction rather than spinning the nose round
    if (beta > M_PIf / 2.0f) {
        beta -= M_PIf;
    } else if (beta < -M_PIf / 2.0f) {
        beta += M_PIf;
    }

    // the nose should sit at +offset (counter-clockwise) from the travel direction
    const float error = beta + offset;
    return RADIANS_TO_DEGREES(gainPerSecond * fade * error);
}

STATIC_UNIT_TESTED void bankCompute(const bankModeConfig_t *config, const bankInputs_t *in, bankState_t *state, bankOutputs_t *out)
{
    const float deadband = config->stickDeadband / 100.0f;
    const float thrustToWeight = config->thrustToWeight / 100.0f;

    float rollStick = bankShapeStick(in->roll, deadband);
    float pitchStick = bankShapeStick(in->pitch, deadband);

    // Engaged with the throttle low (arming): no tilt until the right stick has been centred once
    if (!state->started) {
        state->started = true;
        state->tiltLocked = in->throttleLow;
    }
    if (state->tiltLocked) {
        if (rollStick == 0.0f && pitchStick == 0.0f) {
            state->tiltLocked = false;
        } else {
            rollStick = 0.0f;
            pitchStick = 0.0f;
        }
    }

    const float yawStick = bankShapeStick(in->yaw, deadband);

    float phi = rollStick * DEGREES_TO_RADIANS(config->maxBankAngle);
    float theta = atanf(pitchStick * tanf(DEGREES_TO_RADIANS(config->maxTiltAngle)));
    float fz = bankLiftFromStick(in->vertical, thrustToWeight) - 1.0f;

    fz = bankLimitTilt(fz, &theta, &phi, thrustToWeight);
    const float collective = bankCalcCollective(fz, theta, phi, thrustToWeight);

    const float previousOffset = state->headCheckOffset;
    const float offsetTarget = yawStick * DEGREES_TO_RADIANS(config->headCheckRange);
    state->headCheckOffset = bankSlewHeadCheck(previousOffset, offsetTarget, DEGREES_TO_RADIANS(config->headCheckRate), in->dt);
    const float offsetRateDps = in->dt > 0.0f ? RADIANS_TO_DEGREES((state->headCheckOffset - previousOffset) / in->dt) : 0.0f;

    bankRotateTilt(&theta, &phi, state->headCheckOffset);

    const float alignRateDps = bankCalcAlignYawRate(in->dragX, in->dragY, state->headCheckOffset,
        config->yawAlignGain / 10.0f, config->dragThreshold / 1000.0f);

    out->pitchDeg = RADIANS_TO_DEGREES(theta);
    out->rollDeg = RADIANS_TO_DEGREES(phi);
    out->yawRateDps = constrainf(alignRateDps + offsetRateDps, -(float)config->yawMaxRate, (float)config->yawMaxRate);
    out->collective = in->throttleLow ? 0.0f : collective;
}

static void bankReset(void)
{
    bankState.headCheckOffset = 0.0f;
    bankState.filtersValid = false;
    bankState.lastUpdateUs = 0;
    bankState.started = false;
    bankState.tiltLocked = false;
    bankAngleTargetDeg[FD_ROLL] = 0.0f;
    bankAngleTargetDeg[FD_PITCH] = 0.0f;
    bankYawRateDps = 0.0f;
    bankCollective = 0.0f;
}

void bankModeUpdate(timeUs_t currentTimeUs)
{
    bankModeActive = FLIGHT_MODE(BANK_MODE);
    if (!bankModeActive) {
        bankReset();
        return;
    }

    float dt = BANK_FALLBACK_DT_S;
    bool restartFilters = !bankState.filtersValid;
    if (bankState.lastUpdateUs != 0) {
        dt = cmpTimeUs(currentTimeUs, bankState.lastUpdateUs) * 1e-6f;
        if (dt > BANK_MAX_DT_S) {
            dt = BANK_FALLBACK_DT_S;
            restartFilters = true;
        } else if (dt < BANK_MIN_DT_S) {
            return;
        }
    }
    bankState.lastUpdateUs = currentTimeUs;

    const float dragX = acc.accADC.x * acc.dev.acc_1G_rec;
    const float dragY = acc.accADC.y * acc.dev.acc_1G_rec;
    if (restartFilters) {
        bankState.dragX = dragX;
        bankState.dragY = dragY;
        bankState.filtersValid = true;
    } else {
        const float rc = 1.0f / (2.0f * M_PIf * BANK_DRAG_FILTER_HZ);
        const float alpha = dt / (rc + dt);
        bankState.dragX += alpha * (dragX - bankState.dragX);
        bankState.dragY += alpha * (dragY - bankState.dragY);
    }

    const bankInputs_t inputs = {
        .roll = getRcDeflection(FD_ROLL),
        .pitch = getRcDeflection(FD_PITCH),
        .yaw = getRcDeflection(FD_YAW),
        .vertical = (rcCommand[THROTTLE] - PWM_RANGE_MIN) / (float)PWM_RANGE,
        .throttleLow = calculateThrottleStatus() == THROTTLE_LOW,
        .dragX = bankState.dragX,
        .dragY = bankState.dragY,
        .dt = dt,
    };
    bankOutputs_t outputs;
    bankCompute(bankModeConfig(), &inputs, &bankState, &outputs);

    bankAngleTargetDeg[FD_ROLL] = outputs.rollDeg;
    bankAngleTargetDeg[FD_PITCH] = outputs.pitchDeg;
    bankYawRateDps = outputs.yawRateDps;
    bankCollective = outputs.collective;
}

bool bankModeIsActive(void)
{
    return bankModeActive;
}

float bankModeGetAngleTarget(int axis)
{
    return bankAngleTargetDeg[axis];
}

float bankModeGetYawRate(void)
{
    return bankYawRateDps;
}

float bankModeGetCollective(void)
{
    return bankCollective;
}

#endif // USE_BANK_MODE
