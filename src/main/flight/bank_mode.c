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

#include "flight/imu.h"

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
 *   throttle  -> forward speed command along the travel direction, realised as tilt theta plus collective.
 *                The bottom of the stick is zero speed (hard brake, and the pose for arming), the top is the steady
 *                speed at the maximum tilt. There is no reverse.
 *   yaw       -> head check: nose offset from the travel direction, springs back to 0 at centre.
 *   pitch     -> vertical lift command (not raw throttle). The bottom of the stick gives no lift at all (free
 *                fall), the centre gives 1 g (thrust balances weight), and the top gives the maximum thrust,
 *                all of it as lift, whatever multiple of the weight that is.
 *   roll      -> bank angle phi, i.e. a turn in the horizontal plane. Centre = none.
 *
 * Arming is with the left stick at the bottom (zero speed) and the right stick wherever the lift should start.
 * If the mode is engaged with the throttle low and the roll stick off-centre, the tilt stays at zero until
 * the roll stick has been centred once.
 *
 * Each update, while BANK_MODE is active:
 *
 *   phi   = rollStick * maxBankAngle
 *   theta = atan((vTarget + Kp * (vTarget - v)) / (1 + Fz))   // clamped to the maximum tilt, see below
 *   Fz    = lift(pitchStick) - 1           // lift: 0 at the bottom, 1 at the centre, Tmax at the top, linear between
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
 *   Thrust-to-weight (Tmax) is learnt in flight, so the centre of the lift stick holds the height without having to
 *   guess it. The thrust is along body Z, so the accelerometer's Z reading is the thrust in g whatever the tilt:
 *     Tmax = lowpass(accZ / collective)     // learnt only while armed and T is between 1 g and 1 g + a small window
 *   Learning only near 1 g makes Tmax the value that hovers at the centre of the stick, whatever the shape of the
 *   thrust curve. On the ground the accelerometer reads 1 g whatever the collective, so with T = 1 g the estimate
 *   holds, and with the stick above the centre (T > 1 g) it can only fall, until the craft lifts off and the real
 *   thrust is measured. bank_thrust_to_weight is the starting value, and the
 *   estimate is kept within +-30% of it. It is reset on disarm.
 *
 *   Forward speed (throttle stick): the speed is in g of drag, the horizontal thrust that holds it in steady flight,
 *   so no m/s calibration is needed. It comes from the same filtered accelerometer drag as the ball:
 *     v       = -deadband(forwardDrag) / cos(measured travel-frame pitch)
 *     vTarget = speedStick * tan(maxTiltAngle)         // 0..1, so never negative
 *     Fx      = vTarget + Kp * (vTarget - v)             // feedforward + P, no integrator
 *   At the bottom of the stick Fx = -Kp * v, an artificial drag that tilts the craft back in proportion to its speed
 *   until it stops. At steady speed the error is 0 and the tilt is the one that holds vTarget, as before.
 *
 *   Head check: the offset slews towards yawStick * headCheckRange. The thrust direction is built
 *   in the travel frame and rotated into the body frame by the offset, so the earth-frame thrust
 *   direction, and so the travel direction, is not changed by looking away. Tilt magnitude and
 *   collective are invariant under that rotation.
 *
 *   Coordinated turn, the "ball": for a banked turn tan(phi) = v * omega / g, so omega = g * tan(phi) / v
 *   needs the speed. Instead of computing that, yaw is driven to keep the ball of a turn-and-slip
 *   indicator centred, which is the same condition with v eliminated. The ball is the lateral
 *   specific force in the travel frame. The accelerometer measures thrust (along body Z) plus drag,
 *   so its X/Y reading is rotor drag, roughly proportional to velocity and opposite to it, and its
 *   lateral part is a direct slip signal:
 *     ball = -direction * deadband(dragX * sin(offset) + dragY * cos(offset))
 *     yawRate = Kp * ball + integral(Ki * ball) + headCheckSlewRate
 *   direction is +1 forwards and flips to -1 for clear rearward drag, so the tail follows the travel
 *   when reversing, like a car. The bank tilts the thrust sideways, the craft starts to slip, the ball
 *   moves, and the yaw follows until the ball is centred again. At that point the thrust is beside
 *   the nose and the turn rate is whatever the speed needs. The integral term holds that rate.
 *
 * Limitations:
 *   - The ball only sees aerodynamic side force, so at low speed there is little signal; the
 *     deadband stops accelerometer trim errors from being integrated into a slow spin.
 *   - The loop gain grows with speed, so Ki may need to be lower for fast flight.
 *   - The speed is only as good as the linear rotor-drag model. Body drag adds a v^2 term at high speed, so the
 *     stick-to-speed scale is non-linear, but it is monotonic and zero at zero speed.
 *   - Accelerometer bias reads as speed. The speed deadband hides small bias; a larger one leaves a slow drift.
 *   - Vertical speed leaks into the body X drag while tilted, and the 2 Hz drag filter adds lag, so a high Kp can
 *     oscillate.
 *   - The head check offset only enters by rotating the ball, it is not tracked as an angle.
 *   - The centre of the lift stick holds the vertical acceleration at zero, not the vertical speed: a climb or a
 *     descent carries on at the centre, slowed only by drag. Vertical speed is not observable from the
 *     accelerometer, because the vertical drag and the thrust share the Z axis.
 *   - Vertical drag reads as less thrust, so a steady climb at the centre lowers the estimate slightly.
 *   - Tmax is only learnt near level hover. Away from it the thrust is taken as linear in the collective, so with
 *     a convex thrust curve a hard bank or a full-lift climb gets a little less thrust than asked for.
 *
 * OPEN: interaction with HEADFREE and arming; coexistence with ALT_HOLD/POS_HOLD (they currently cancel BANK).
 */

#define BANK_DRAG_FILTER_HZ     2.0f    // cutoff of the accelerometer drag filter
#define BANK_MIN_DT_S           0.0005f
#define BANK_MAX_DT_S           0.2f    // longer gaps restart the filters
#define BANK_FALLBACK_DT_S      0.01f
#define BANK_MIN_TILT_COS       1e-3f
#define BANK_MIN_FLIP_DRAG      0.01f   // g of rearward drag before the ball sign starts to flip
#define BANK_MIN_LIFT           0.1f    // lowest lift, in g, used to turn a horizontal force into a tilt
#define BANK_TW_FILTER_HZ       0.2f    // cutoff of the thrust-to-weight learning filter
#define BANK_TW_LEARN_WINDOW    0.3f    // g of thrust above 1 g that is still learnt from
#define BANK_TW_LEARN_TOLERANCE 1e-3f   // g of thrust below 1 g that is still learnt from (rounding at the centre)
#define BANK_TW_RANGE           0.3f    // the estimate stays within this fraction of the configured value

static bankState_t bankState;

static FAST_DATA_ZERO_INIT float bankAngleTargetDeg[RP_AXIS_COUNT];
static FAST_DATA_ZERO_INIT float bankYawRateDps;
static FAST_DATA_ZERO_INIT float bankCollective;
static FAST_DATA_ZERO_INIT float bankThrustToWeight;
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

// Vertical lift, in multiples of the weight, for the lift stick position (0 bottom, 1 top):
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

// Learns the thrust-to-weight ratio from the accelerometer. The thrust is along body Z, so accZ is the thrust in g
// whatever the tilt, and accZ / collective is the thrust a full collective would give, measured at this operating
// point. thrustDemand and collective are those of the previous update, the ones the accelerometer is responding to.
// Only learnt between 1 g and 1 g + BANK_TW_LEARN_WINDOW of thrust: the neutral lift stick, nearly level. Returns the
// new estimate, kept within BANK_TW_RANGE of the configured prior.
STATIC_UNIT_TESTED float bankLearnThrustToWeight(float estimate, float prior, float accZ, float collective, float thrustDemand, float dt)
{
    if (thrustDemand < 1.0f - BANK_TW_LEARN_TOLERANCE || thrustDemand > 1.0f + BANK_TW_LEARN_WINDOW) {
        return estimate;
    }
    const float rc = 1.0f / (2.0f * M_PIf * BANK_TW_FILTER_HZ);
    const float alpha = dt / (rc + dt);
    estimate += alpha * (accZ / collective - estimate);
    return constrainf(estimate, fmaxf(prior * (1.0f - BANK_TW_RANGE), 1.0f), prior * (1.0f + BANK_TW_RANGE));
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

// Subtractive deadband: values within +-band become 0, and larger ones move towards 0 by band,
// so the output is continuous (no step at the edge of the band).
STATIC_UNIT_TESTED float bankDeadband(float value, float band)
{
    if (value > band) {
        return value - band;
    }
    if (value < -band) {
        return value + band;
    }
    return 0.0f;
}

// Rotates the body-frame drag (dragX forward, dragY left) into the travel frame. The nose sits at +offset
// (counter-clockwise) from the travel direction, so the drag is rotated back by offset.
// forward is negative when travelling forwards; lateral is negative when the travel is to the left of the nose.
STATIC_UNIT_TESTED void bankDragToTravelFrame(float dragX, float dragY, float offset, float *forward, float *lateral)
{
    const float c = cosf(offset);
    const float s = sinf(offset);
    *forward = dragX * c - dragY * s;
    *lateral = dragX * s + dragY * c;
}

// The "ball" of a turn-and-slip indicator: the lateral specific force (g) seen from the head-check frame,
// signed so that a positive value asks for a counter-clockwise (positive) yaw. Zero when the craft is
// neither slipping nor skidding. dragX/dragY are the filtered accelerometer X/Y readings in g.
//
// A multirotor's thrust is along body Z, so the accelerometer's X/Y reading is aerodynamic drag, which is
// roughly proportional to velocity and opposite to it. Its lateral part is therefore a direct slip signal:
// no velocity estimate or speed is needed, and low drag (low speed) simply gives a small ball.
STATIC_UNIT_TESTED float bankCalcBall(float dragX, float dragY, float offset, float deadband)
{
    float forwardDrag;
    float lateral;
    bankDragToTravelFrame(dragX, dragY, offset, &forwardDrag, &lateral);
    lateral = bankDeadband(lateral, deadband);

    // Forwards by default, so sideways slip from rest turns the nose towards it. Only clear backwards travel flips
    // the sign, so the tail follows the travel like a reversing car instead of the loop going unstable.
    const float flipStart = fmaxf(deadband, BANK_MIN_FLIP_DRAG);
    const float backwards = constrainf((forwardDrag - flipStart) / flipStart, 0.0f, 1.0f);
    const float direction = 1.0f - 2.0f * backwards;

    return -direction * lateral;
}

// PI controller on the ball. The proportional part damps, the integral part holds the yaw rate a steady
// turn needs, so the ball settles exactly at the centre. Returns the yaw rate in degrees per second.
// Forward speed in the travel direction, in g of drag (the horizontal thrust that holds this speed in steady
// flight), positive when travelling forwards. forwardDrag is the travel-frame drag from bankDragToTravelFrame().
// The accelerometer sees drag along the tilted body plane, so it is divided by cos(theta) to get the horizontal part.
STATIC_UNIT_TESTED float bankCalcForwardSpeed(float forwardDrag, float measuredTheta, float deadband)
{
    const float cosTheta = fmaxf(cosf(measuredTheta), BANK_MIN_TILT_COS);
    return bankDeadband(-forwardDrag, deadband) / cosTheta;
}

// Forward tilt that makes the craft hold the target speed. Feedforward plus P, no integrator:
//   Fx = vTarget + kp * (vTarget - v)      horizontal force in g
//   theta = atan(Fx / lift)                lift = 1 + Fz
// At the bottom of the speed stick (vTarget = 0) this is Fx = -kp * v: extra drag that brakes the craft until it stops.
// At steady speed vTarget the drag equals the feedforward, so the error is 0 and theta is the tilt for vTarget.
STATIC_UNIT_TESTED float bankVelocityToTilt(float vTarget, float v, float kp, float fz, float maxTilt)
{
    const float fx = vTarget + kp * (vTarget - v);
    const float lift = fmaxf(1.0f + fz, BANK_MIN_LIFT);
    return constrainf(atanf(fx / lift), -maxTilt, maxTilt);
}

STATIC_UNIT_TESTED float bankStepBallController(float ball, float kp, float ki, float maxRate, float dt, float *integrator)
{
    *integrator = constrainf(*integrator + ki * ball * dt, -maxRate, maxRate);
    return constrainf(kp * ball + *integrator, -maxRate, maxRate);
}

STATIC_UNIT_TESTED void bankCompute(const bankModeConfig_t *config, const bankInputs_t *in, bankState_t *state, bankOutputs_t *out)
{
    const float deadband = config->stickDeadband / 100.0f;

    const float thrustToWeightPrior = config->thrustToWeight / 100.0f;
    if (state->thrustToWeight == 0.0f) {
        state->thrustToWeight = thrustToWeightPrior;
    }
    if (in->armed) {
        state->thrustToWeight = bankLearnThrustToWeight(state->thrustToWeight, thrustToWeightPrior,
            in->accZ, state->collective, state->thrustDemand, in->dt);
    }
    const float thrustToWeight = state->thrustToWeight;

    float rollStick = bankShapeStick(in->roll, deadband);
    float speedStick = bankShapeStick(in->speed, deadband);     // 0..1, the deadband is at the bottom

    // Engaged with the throttle low (arming): no tilt until the roll stick has been centred once
    if (!state->started) {
        state->started = true;
        state->tiltLocked = in->throttleLow;
    }
    if (state->tiltLocked) {
        if (rollStick == 0.0f && speedStick == 0.0f) {
            state->tiltLocked = false;
        } else {
            rollStick = 0.0f;
            speedStick = 0.0f;
        }
    }

    const float yawStick = bankShapeStick(in->yaw, deadband);

    float phi = rollStick * DEGREES_TO_RADIANS(config->maxBankAngle);
    // The lift stick springs back to the centre (hover), so it gets the same deadband there
    const float liftStick = 0.5f * (bankShapeStick(2.0f * in->lift - 1.0f, deadband) + 1.0f);
    float fz = bankLiftFromStick(liftStick, thrustToWeight) - 1.0f;

    // Throttle stick = forward speed along the travel direction (in g of drag, full stick = steady flight at max tilt).
    // The measured speed comes from the drag and the measured tilt, both seen from the travel frame.
    const float maxTilt = DEGREES_TO_RADIANS(config->maxTiltAngle);
    float measuredTheta = DEGREES_TO_RADIANS(in->measuredPitchDeg);
    float measuredPhi = DEGREES_TO_RADIANS(in->measuredRollDeg);
    bankRotateTilt(&measuredTheta, &measuredPhi, -state->headCheckOffset);
    float forwardDrag;
    float lateralDrag;
    bankDragToTravelFrame(in->dragX, in->dragY, state->headCheckOffset, &forwardDrag, &lateralDrag);
    const float speed = bankCalcForwardSpeed(forwardDrag, measuredTheta, config->speedDeadband / 1000.0f);
    const float targetSpeed = speedStick * tanf(maxTilt);
    float theta = bankVelocityToTilt(targetSpeed, speed, config->speedKp / 100.0f, fz, maxTilt);

    fz = bankLimitTilt(fz, &theta, &phi, thrustToWeight);
    const float collective = bankCalcCollective(fz, theta, phi, thrustToWeight);
    state->collective = collective;
    state->thrustDemand = collective * thrustToWeight;

    const float previousOffset = state->headCheckOffset;
    const float offsetTarget = yawStick * DEGREES_TO_RADIANS(config->headCheckRange);
    state->headCheckOffset = bankSlewHeadCheck(previousOffset, offsetTarget, DEGREES_TO_RADIANS(config->headCheckRate), in->dt);
    const float offsetRateDps = in->dt > 0.0f ? RADIANS_TO_DEGREES((state->headCheckOffset - previousOffset) / in->dt) : 0.0f;

    bankRotateTilt(&theta, &phi, state->headCheckOffset);

    const float maxYawRate = config->yawMaxRate;
    float coordinationRateDps = 0.0f;
    if (liftStick == 0.0f) {
        // no lift: on the ground or falling, there is nothing to turn
        state->ballIntegrator = 0.0f;
    } else {
        const float ball = bankCalcBall(in->dragX, in->dragY, state->headCheckOffset, config->ballDeadband / 1000.0f);
        coordinationRateDps = bankStepBallController(ball, config->ballKp, config->ballKi, maxYawRate, in->dt, &state->ballIntegrator);
    }

    out->pitchDeg = RADIANS_TO_DEGREES(theta);
    out->rollDeg = RADIANS_TO_DEGREES(phi);
    out->yawRateDps = constrainf(coordinationRateDps + offsetRateDps, -maxYawRate, maxYawRate);
    out->collective = collective;   // no lift gives zero by itself
}

static void bankReset(void)
{
    bankState.headCheckOffset = 0.0f;
    bankState.ballIntegrator = 0.0f;
    bankState.collective = 0.0f;    // nothing to learn from until the mode has set a collective again
    bankState.thrustDemand = 0.0f;
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
    // The learnt thrust-to-weight survives the mode being switched off and on, but not a disarm (new battery, new config)
    if (!ARMING_FLAG(ARMED)) {
        bankState.thrustToWeight = 0.0f;
    }
    bankThrustToWeight = bankState.thrustToWeight != 0.0f ? bankState.thrustToWeight : bankModeConfig()->thrustToWeight / 100.0f;

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
        .speed = (rcCommand[THROTTLE] - PWM_RANGE_MIN) / (float)PWM_RANGE,   // 0 at the bottom, 1 at the top
        .yaw = getRcDeflection(FD_YAW),
        .lift = 0.5f * (getRcDeflection(FD_PITCH) + 1.0f),
        .throttleLow = calculateThrottleStatus() == THROTTLE_LOW,
        .armed = ARMING_FLAG(ARMED),
        .accZ = acc.accADC.z * acc.dev.acc_1G_rec,
        .dragX = bankState.dragX,
        .dragY = bankState.dragY,
        .measuredPitchDeg = attitude.raw[FD_PITCH] * 0.1f,
        .measuredRollDeg = attitude.raw[FD_ROLL] * 0.1f,
        .dt = dt,
    };
    bankOutputs_t outputs;
    bankCompute(bankModeConfig(), &inputs, &bankState, &outputs);

    bankAngleTargetDeg[FD_ROLL] = outputs.rollDeg;
    bankAngleTargetDeg[FD_PITCH] = outputs.pitchDeg;
    bankYawRateDps = outputs.yawRateDps;
    bankCollective = outputs.collective;
    bankThrustToWeight = bankState.thrustToWeight;
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

float bankModeGetThrustToWeight(void)
{
    return bankThrustToWeight;
}

#endif // USE_BANK_MODE
