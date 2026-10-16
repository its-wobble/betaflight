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
 * Attitude only (gyro/ACC): no GPS, baro or velocity estimate is needed.
 * Forces are in multiples of the craft's weight (m*g = 1), so the thrust limit is Tmax = T/W.
 * Angles: roll is positive to the right, pitch is positive nose-down (forward), as in attitude.raw[].
 * Body axes are X forward, Y left, Z up, and yaw rates are positive counter-clockwise.
 *
 * Stick mapping (mode 2):
 *   throttle  -> forward speed command along the travel direction, realised as tilt theta plus collective.
 *                The lower half is a speed: zero at the bottom (hard brake, and the pose for arming) to
 *                bank_mid_speed at the centre; only the bottom third brakes a craft that is too fast, above it the
 *                craft coasts down. The upper half is a forward force, from what the centre gives to all the thrust
 *                allows at the top, as at full throttle in acro, so both modes reach the same top speed. There is no
 *                reverse.
 *   yaw       -> bank angle phi, i.e. a turn in the horizontal plane. Centre = none.
 *   pitch     -> vertical lift command (not raw throttle). The bottom of the stick gives no lift at all (free
 *                fall), the centre gives 1 g (thrust balances weight), and the top gives the maximum thrust,
 *                all of it as lift, whatever multiple of the weight that is.
 *   roll      -> head check: nose offset from the travel direction, springs back to 0 at centre.
 *
 * Arming is with the left stick at the bottom (zero speed) and the right stick wherever the lift should start.
 * If the mode is engaged with the throttle low and the bank (yaw) stick off-centre, the tilt stays at zero
 * until the bank stick has been centred once.
 *
 * Each update, while BANK_MODE is active:
 *
 *   phi   = rollStick * maxBankAngle
 *   theta = atan(Fx / (1 + Fz))            // Fx from the speed stick, see below; braking no steeper than bank_max_brake_angle
 *   Fz    = lift(pitchStick) - 1           // lift: 0 at the bottom, 1 at the centre, Tmax at the top; above the centre
 *                                           // 1 + x + (Tmax - 2) * x^3, so both halves start at 1 g per half stick
 *
 *   Thrust saturation, in priority order: Fz first, then bank (phi), forward force (theta) last. Tmax is the
 *   thrust-to-weight times the thrust trim, so in fast flight, where the props make less thrust, the tilt is less.
 *     Fz   = min(Fz, Tmax - 1)
 *     Ttilt = (1 + Fz) + 0.75 * (Tmax - (1 + Fz))      // a quarter of the spare thrust is kept in reserve
 *     cosTiltMin = (1 + Fz) / Ttilt                     // T <= Ttilt  <=>  cos(theta) * cos(phi) >= cosTiltMin
 *     phi   = min(phi, acos(cosTiltMin))                // bank takes what it needs first
 *     phi   = min(phi, max(30 deg, atan(0.8 * yawMaxRate * v / turnGain)))   // and no steeper than the yaw can follow
 *     theta = min(theta, acos(cosTiltMin / cos(phi)))   // forward tilt gets what remains
 *
 *   The collective is always computed from the measured tilt, never taken from the stick, so the vertical force is the
 *   lift stick's whatever the speed stick and through every change of attitude:
 *     T = (1 + Fz) / max(cos(measured tilt), cosTiltMin),  collective = (T / Tmax) ^ (1 / 1.5)   // thrust ~ collective ^ 1.5
 *
 *   Thrust-to-weight (Tmax) is the setting bank_thrust_to_weight, fixed: nothing is learnt in flight. Set it from a
 *   steady hover at the centre of the lift stick, Tmax = 1 / collective ^ 1.5, so the centre gives 1 g. As the battery
 *   drains the props make less thrust for the same collective and the centre sinks slowly.
 *   A fast thrust trim (bankUpdateThrustTrim()) corrects the thrust from the measured vertical force: it takes off quick
 *   excesses, such as braking from speed, and adds what fast forward flight loses while the craft is sinking.
 *
 *   Forward speed (throttle stick): the speed is in g of drag, the horizontal thrust that holds it in steady flight,
 *   so no m/s calibration is needed. It comes from the same filtered accelerometer drag as the ball. The rotor drag
 *   grows as sqrt(thrust), so the drag is divided by sqrt(T) first, and a hard bank does not read as more speed:
 *     v       = -deadband(forwardDrag)                 // forwardDrag levelled with the measured attitude, see the ball
 *     vTarget = 2 * min(speedStick, 0.5) * midSpeed      // lower half: 0 to bank_mid_speed
 *     Fx      = vTarget + Kp * (vTarget - v)             // feedforward + P, no integrator; above a third of the stick
 *                                                        // the P term only adds (max(vTarget - v, 0)): no braking
 *     Fx      = Fx(centre) + (2 * speedStick - 1) * (FxMax - Fx(centre))   // upper half: a force, no speed hold
 *     FxMax   = (1 + Fz) * tan(thetaMax)                 // the most forward force the thrust allows
 *   At the bottom of the stick Fx = -Kp * v, an artificial drag that tilts the craft back in proportion to its speed
 *   until it stops. At steady speed the error is 0 and the tilt is the one that holds vTarget. The force over the stick
 *   is continuous at the centre; at a third it steps down by Kp * (v - vTarget) when the craft is too fast: the brake.
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
 *   lateral part is a direct slip signal. The drag is first levelled with the measured attitude (pitched and banked,
 *   part of the forward drag lands on body Y), then rotated into the travel frame:
 *     levelX = dragX / cos(pitch),  levelY = (dragY - levelX * sin(roll) * sin(pitch)) / cos(roll)
 *     ball = -direction * deadband(levelX * sin(offset) + levelY * cos(offset))
 *     yawRate = Kp * ball + integral(Ki * ball) + headCheckSlewRate
 *   direction is +1 forwards and flips to -1 for clear rearward drag, so the tail follows the travel
 *   when reversing, like a car. The bank tilts the thrust sideways, the craft starts to slip, the ball
 *   moves, and the yaw follows until the ball is centred again. At that point the thrust is beside
 *   the nose and the turn rate is whatever the speed needs. The integral term holds that rate. That rate is
 *   proportional to tan(phi), so when the bank is reduced the integral is scaled down with it (and cleared when the
 *   bank stick centres) instead of being left for the ball to unwind.
 *   The ball alone reacts only once the craft has slipped, which in a steep bank at low speed is a skid. So the rate
 *   the bank needs is also fed forward: omega = turnGain * tan(phi) / v, with v the speed in g of drag and
 *   turnGain = g * (g of drag per m/s), a constant of the craft. It is learnt in steady turns from what the ball
 *   integrator still has to add, and like the thrust-to-weight it is reset on disarm.
 *
 * Limitations:
 *   - The ball only sees aerodynamic side force, so at low speed there is little signal; the
 *     deadband stops accelerometer trim errors from being integrated into a slow spin.
 *   - The loop gain grows with speed, so Ki may need to be lower for fast flight.
 *   - The speed is only as good as the linear rotor-drag model. Body drag adds a v^2 term at high speed, so the
 *     stick-to-speed scale is non-linear, but it is monotonic and zero at zero speed.
 *   - Accelerometer bias reads as speed. The speed deadband hides small bias; a larger one leaves a slow drift.
 *   - The 2 Hz drag filter adds lag, so a high Kp can oscillate. A climb or a sink with the craft tilted puts air
 *     across the rotors too, and that drag reads as speed or slip: there is no vertical speed estimate to take it out.
 *   - The head check offset only enters by rotating the ball, it is not tracked as an angle.
 *   - The lift stick is a force, not a vertical speed: the centre holds 1 g, so a climb or a sink carries on until the
 *     pilot stops it. Nothing holds the height or the vertical speed (no baro, no vertical speed estimate).
 *   - The thrust per collective is fixed, but in a climb, in fast forward flight and as the battery sags the props
 *     make less of it. The thrust trim adds it back only while the craft reads below 1 g (it cannot tell the ground
 *     from missing thrust at 1 g), so the centre of the lift stick still sinks a little there.
 *   - The thrust curve is taken as collective ^ 1.5. A craft whose curve differs gets a slightly different thrust
 *     away from the hover, where Tmax was measured.
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
#define BANK_TRIM_MIN_TILT_COS  0.25f   // the thrust trim is not measured when tilted more than 75 degrees
#define BANK_TRIM_FILTER_HZ     3.0f    // cutoff of the fast thrust trim
#define BANK_TRIM_LAG_HZ        10.0f   // the motors' and the accelerometer filter's lag, put on the modelled thrust
#define BANK_TRIM_MAX           2.0f    // the trim takes thrust off by up to this factor (see bankUpdateThrustTrim())
#define BANK_TRIM_MIN           0.2f    // and adds it, while the craft is measurably sinking, by up to 1 / this
#define BANK_TRIM_MIN_LIFT      0.3f    // g of modelled vertical force below which the trim is not measured (free fall, idle)
#define BANK_TRIM_LEAK_S        1.0f    // time constant the trim goes back to 1 with, so the ground cannot hold it off 1
#define BANK_TRIM_ADD_LEAK_S    5.0f    // the same for a trim that adds thrust, slower so it holds through fast flight
#define BANK_BRAKE_STICK        (1.0f / 3.0f)   // below this speed stick a craft faster than the stick's speed is braked
#define BANK_TILT_THRUST_SHARE  0.75f  // share of the thrust above the vertical force that bank and forward tilt may use
#define BANK_TURN_RATE_SHARE    0.8f    // share of bank_yaw_max_rate a coordinated turn may need, the rest is for the ball
#define BANK_MIN_BANK_LIMIT_DEG 30.0f   // the speed limit on the bank never goes below this, so a bank from the hover still moves
#define BANK_THRUST_EXPONENT    1.5f    // thrust ~ collective ^ this: props make thrust ~ rpm^2, and rpm is not linear in the command
#define BANK_DRAG_MIN_THRUST    0.25f   // g: the lowest thrust the rotor drag is normalised by
#define BANK_TURN_GAIN_PRIOR    19.0f   // deg/s * g: g * (g of drag per m/s) of a 5 inch, the starting turn gain
#define BANK_TURN_GAIN_RANGE    4.0f    // the learnt turn gain stays within this factor of the prior
#define BANK_TURN_MIN_SPEED     0.1f    // g of drag: below this the feedforward fades out to nothing at rest
#define BANK_TURN_LEARN_TAU_S   2.0f    // how fast the ball integrator is moved into the turn gain
#define BANK_TURN_LEARN_MIN_TAN 0.18f   // tan(10 degrees): the turn gain is only learnt in a real bank

static bankState_t bankState;

static FAST_DATA_ZERO_INIT bankOutputs_t bankOutputs;     // read by the PID loop and the mixer

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
// none at the bottom, 1 g at the centre, and all of the maximum thrust at the top. Below the centre it is linear. Above
// it starts with the same slope, 1 g per half stick, and curves up to the maximum: lift = 1 + x + (T/W - 2) * x^3 with x
// from 0 at the centre to 1 at the top. A straight line to a 5 inch's 7 or 8 g would make the upper half of the stick
// seven times as touchy as the lower one, and every twitch around the centre a big jump in thrust.
STATIC_UNIT_TESTED float bankLiftFromStick(float stick, float thrustToWeight)
{
    const float s = constrainf(stick, 0.0f, 1.0f);
    if (s <= 0.5f) {
        return 2.0f * s;
    }
    const float x = 2.0f * s - 1.0f;
    const float tw = fmaxf(thrustToWeight, 1.0f);
    if (tw <= 2.0f) {
        return 1.0f + x * (tw - 1.0f);     // not even 1 g of reserve: no room for the curve
    }
    return 1.0f + x + (tw - 2.0f) * x * x * x;
}

// Limits the tilt so the collective needed for the vertical force stays within thrust-to-weight.
// Fz has priority, then bank (phi), then forward tilt (theta). Returns the (possibly capped) Fz.
// The tilt only gets BANK_TILT_THRUST_SHARE of the thrust left over after the vertical force. The priority order alone
// would share out all of it, but then a bank set steeper than the thrust allows rides the limit with the collective at
// full, and the mixer has no motor that can speed up: the attitude loop loses half its authority just when the craft
// is banked hardest (in the simulator, a full bank at medium speed overshot to 84 deg and rolled over). The reserve is
// also what a thrust-to-weight set too high is corrected with.
// The cosine of the steepest total tilt the thrust allows for the vertical force fz (already capped), with the reserve.
static float bankTiltCosMin(float fz, float thrustToWeight)
{
    const float tw = fmaxf(thrustToWeight, 1.0f);
    const float tiltThrust = (1.0f + fz) + BANK_TILT_THRUST_SHARE * (tw - (1.0f + fz));
    return (1.0f + fz) / fmaxf(tiltThrust, BANK_MIN_TILT_COS);
}

STATIC_UNIT_TESTED float bankLimitTilt(float fz, float *theta, float *phi, float thrustToWeight)
{
    const float tw = fmaxf(thrustToWeight, 1.0f);
    fz = constrainf(fz, -1.0f, tw - 1.0f);

    const float cosTiltMin = bankTiltCosMin(fz, tw);

    const float phiMax = acosf(constrainf(cosTiltMin, 0.0f, 1.0f));
    *phi = constrainf(*phi, -phiMax, phiMax);

    const float cosPhi = fmaxf(cosf(*phi), BANK_MIN_TILT_COS);
    const float thetaMax = acosf(constrainf(cosTiltMin / cosPhi, 0.0f, 1.0f));
    *theta = constrainf(*theta, -thetaMax, thetaMax);

    return fz;
}

// The thrust curve: thrust (g) = thrustToWeight * collective ^ BANK_THRUST_EXPONENT. A straight line would make a
// thrust-to-weight measured at the hover wrong in a 2 g turn.
STATIC_UNIT_TESTED float bankThrustFromCollective(float collective, float thrustToWeight)
{
    return thrustToWeight * powf(fmaxf(collective, 0.0f), BANK_THRUST_EXPONENT);
}

STATIC_UNIT_TESTED float bankCollectiveFromThrust(float thrust, float thrustToWeight)
{
    return powf(constrainf(thrust / fmaxf(thrustToWeight, 1.0f), 0.0f, 1.0f), 1.0f / BANK_THRUST_EXPONENT);
}

// Collective as a fraction of the maximum, from the vertical force and the cosine of the tilt.
STATIC_UNIT_TESTED float bankCalcCollective(float fz, float cosTilt, float thrustToWeight)
{
    return bankCollectiveFromThrust((1.0f + fz) / fmaxf(cosTilt, BANK_MIN_TILT_COS), thrustToWeight);
}

// The cosine of the tilt the collective is set for: the measured one, so the vertical force is the lift stick's through
// every change of attitude. Set for the commanded tilt, rolling into a bank at speed (the pitch target going from 50 to
// 0 degrees and the bank target from 0 to 80 at once) asked for the collective of an 80 degree bank while the craft was
// still nearly level: 4 to 5 g up for a moment, and a 3 to 5 m/s climb on every turn entry in the owner's flight. Never
// steeper than the thrust allows (with its reserve), so an attitude overshoot does not take the collective to full.
STATIC_UNIT_TESTED float bankCollectiveTiltCos(float measuredTiltCos, float fz, float thrustToWeight)
{
    return fmaxf(measuredTiltCos, bankTiltCosMin(fz, thrustToWeight));
}

// The vertical specific force, in g: the accelerometer reading projected onto the earth vertical with the measured
// attitude. 1 g when the craft is neither climbing nor sinking faster, whatever the tilt and the drag.
STATIC_UNIT_TESTED float bankCalcVerticalForce(float accX, float accY, float accZ, float pitchDeg, float rollDeg)
{
    const float theta = DEGREES_TO_RADIANS(pitchDeg);
    const float phi = DEGREES_TO_RADIANS(rollDeg);
    const float cosTheta = cosf(theta);
    return -sinf(theta) * accX + cosTheta * sinf(phi) * accY + cosTheta * cosf(phi) * accZ;
}

// Fast thrust trim: how much more vertical force the craft makes than the collective was set for, at
// BANK_TRIM_FILTER_HZ, dividing the thrust the collective asks for. It takes off two quick balloons with the lift stick
// centred, where the fixed thrust-to-weight is wrong for a moment:
//   - braking from speed, the rotors tilt back into the air and make up to 1.6 times the modelled thrust (5 m/s of
//     climb in the simulator);
//   - rolling into a bank, the collective rises by 1 / cos(tilt) at once, while the craft is still nearly level.
// And it adds the thrust fast forward flight loses: at 40 m/s the simulator's props make a third of the modelled
// thrust, and without the trim a full speed stick flew into the ground in a few seconds.
// verticalForce is measured (bankCalcVerticalForce()); modelVertical is the model's vertical force for the
// collective sent at the measured tilt, lagged like the motors (state->trimModelVertical).
// Thrust is only added (trim < 1) while the craft reads below 1 g. On the ground with the lift stick above the centre
// the craft reads 1 g, less than it asks, and a trim that added thrust there would launch it on lift-off; below 1 g it
// is not resting on anything. It always leaks back to 1: resting on the ground the accelerometer reads 1 g whatever the
// collective, so any trim measures as right there and would be held off 1 until lift-off. The leak of a trim that adds
// thrust is slower, so it holds through a long fast run. It does not correct a thrust-to-weight set wrong or the
// battery sagging, as long as the craft does not sink below 1 g for it.
// Without a measurement (mixer saturated, tilted past 75 degrees, too little lift to measure) it only leaks.
STATIC_UNIT_TESTED float bankUpdateThrustTrim(float trim, float verticalForce, float modelVertical, bool measurable, float dt)
{
    if (measurable && modelVertical >= BANK_TRIM_MIN_LIFT) {
        float target = verticalForce / modelVertical;
        if (verticalForce >= 1.0f) {
            // Below 1 g the craft is not resting on anything, so less vertical force than modelled is thrust lost (fast
            // forward flight, a climb). At 1 g it may be the ground holding it, which hides missing thrust: the trim
            // only goes back up towards 1 there
            target = fmaxf(target, fminf(trim, 1.0f));
        }
        trim += pt1FilterGain(BANK_TRIM_FILTER_HZ, dt) * (constrainf(target, BANK_TRIM_MIN, BANK_TRIM_MAX) - trim);
    }
    return trim + dt / (trim < 1.0f ? BANK_TRIM_ADD_LEAK_S : BANK_TRIM_LEAK_S) * (1.0f - trim);
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
    if (offset == 0.0f) {
        return;
    }
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

// Levels the body-frame drag (dragX forward, dragY left): returns the horizontal drag along the nose's heading
// (forward) and to its left (left), using the measured attitude (pitch positive nose-down, roll positive right).
// The rotor drag lies in the rotor plane and opposes the velocity there. Taking the velocity as horizontal, the
// heading's forward axis is (cos(pitch), sin(roll) * sin(pitch), ...) and its left axis (0, cos(roll), ...) in the body
// frame, so a pitched and banked craft sees part of its forward drag on body Y. Without this, that part would read as
// slip and the ball would centre with the nose off the track. A climb or a sink also puts air across a tilted rotor
// plane; there is no vertical speed estimate, so that share stays in.
STATIC_UNIT_TESTED void bankDragToLevelFrame(float dragX, float dragY, float pitch, float roll, float *forward, float *left)
{
    *forward = dragX / fmaxf(cosf(pitch), BANK_MIN_TILT_COS);
    *left = (dragY - *forward * sinf(roll) * sinf(pitch)) / fmaxf(cosf(roll), BANK_MIN_TILT_COS);
}

// The rotor drag grows with the rotors' induced velocity, which goes as sqrt(thrust): a 60 degree bank needs 2 g and
// sees 1.4 times the drag at the same speed. Returns the factor that brings the drag back to what it would be at 1 g of
// thrust, so the speed (in g of drag at hover) and the ball mean the same in a hard bank as in level flight.
// thrust is in g; 0 before the mode has set a collective, which leaves the drag as it is.
STATIC_UNIT_TESTED float bankDragThrustScale(float thrust)
{
    if (thrust <= 0.0f) {
        return 1.0f;
    }
    return 1.0f / sqrtf(fmaxf(thrust, BANK_DRAG_MIN_THRUST));
}

// Rotates the levelled drag (dragX forward, dragY left, from bankDragToLevelFrame()) into the travel frame. The nose sits at +offset
// (counter-clockwise) from the travel direction, so the drag is rotated back by offset.
// forward is negative when travelling forwards; lateral is negative when the travel is to the left of the nose.
STATIC_UNIT_TESTED void bankDragToTravelFrame(float dragX, float dragY, float offset, float *forward, float *lateral)
{
    if (offset == 0.0f) {
        *forward = dragX;
        *lateral = dragY;
        return;
    }
    const float c = cosf(offset);
    const float s = sinf(offset);
    *forward = dragX * c - dragY * s;
    *lateral = dragX * s + dragY * c;
}

// The "ball" of a turn-and-slip indicator: the lateral specific force (g) seen from the head-check frame,
// signed so that a positive value asks for a counter-clockwise (positive) yaw. Zero when the craft is
// neither slipping nor skidding. dragX/dragY are the filtered accelerometer X/Y readings in g, levelled by
// bankDragToLevelFrame().
//
// A multirotor's thrust is along body Z, so the accelerometer's X/Y reading is aerodynamic drag, which is
// roughly proportional to velocity and opposite to it. Its lateral part is therefore a direct slip signal:
// no velocity estimate or speed is needed, and low drag (low speed) simply gives a small ball.
STATIC_UNIT_TESTED float bankCalcBall(float dragX, float dragY, float offset, float deadband)
{
    float forwardDrag;
    float lateral;
    bankDragToTravelFrame(dragX, dragY, offset, &forwardDrag, &lateral);
    lateral = fapplyDeadband(lateral, deadband);

    // Forwards by default, so sideways slip from rest turns the nose towards it. Only clear backwards travel flips
    // the sign, so the tail follows the travel like a reversing car instead of the loop going unstable.
    const float flipStart = fmaxf(deadband, BANK_MIN_FLIP_DRAG);
    const float backwards = constrainf((forwardDrag - flipStart) / flipStart, 0.0f, 1.0f);
    const float direction = 1.0f - 2.0f * backwards;

    return -direction * lateral;
}

// The speed asked for by the lower half of the speed stick (0..0.5), in g of drag: from 0 at the bottom to
// bank_mid_speed at the centre in a straight line. The upper half asks for a force, not a speed (bankSpeedForce()).
STATIC_UNIT_TESTED float bankTargetSpeed(float speedStick, float midSpeed)
{
    return 2.0f * constrainf(speedStick, 0.0f, 0.5f) * midSpeed;
}

// The most forward force the thrust allows, in g, for the vertical force fz: lift * tan of the steepest tilt the thrust
// allows with its reserve (bankTiltCosMin()). With the trimmed thrust-to-weight it follows what the props really make:
// on the simulator's five inch 4.4 g at the hover, where the props have all their thrust, and 1.4 g at its 40 m/s top
// speed. It ignores the bank; in a turn the tilt limiter takes the forward force down instead.
STATIC_UNIT_TESTED float bankMaxForwardForce(float fz, float thrustToWeight)
{
    const float tw = fmaxf(thrustToWeight, 1.0f);
    fz = constrainf(fz, -1.0f, tw - 1.0f);
    const float lift = fmaxf(1.0f + fz, BANK_MIN_LIFT);
    const float cosTilt = constrainf(bankTiltCosMin(fz, tw), BANK_MIN_TILT_COS, 1.0f);
    return lift * sqrtf(1.0f - sq(cosTilt)) / cosTilt;
}

// Forward force in the world frame, in g, from the speed stick (0..1) and the speed v (g of drag).
// The lower half is speed control, feedforward plus P, no integrator, towards the speed bankTargetSpeed() asks for:
//   Fx = vTarget + kp * (vTarget - v)
// At steady speed vTarget the drag equals the feedforward, so the error is 0. Only the bottom third brakes: there, a
// craft faster than vTarget gets a negative P term, extra drag that tilts it back until it is down to vTarget (at the
// bottom, until it stops). From a third to the centre the P term only adds force: a craft that is too fast gets the force
// that holds vTarget, and its own drag slows it to vTarget, like coasting off the accelerator. A craft that is too
// slow is driven up to vTarget all the way up to the centre.
// The upper half asks for a force: a straight line from the force the centre gives (with the same P term, so the curve
// of force over the stick is continuous at the centre whatever the speed) to maxForce at the top, so full stick is full
// forward force, as at full throttle in acro, and both modes reach the same top speed. There is no speed hold there.
STATIC_UNIT_TESTED float bankSpeedForce(float speedStick, float v, float midSpeed, float kp, float maxForce)
{
    const float stick = constrainf(speedStick, 0.0f, 1.0f);
    const float vTarget = bankTargetSpeed(stick, midSpeed);
    float error = vTarget - v;
    if (stick > BANK_BRAKE_STICK) {
        error = fmaxf(error, 0.0f);
    }
    const float fx = vTarget + kp * error;
    if (stick <= 0.5f) {
        return fx;
    }
    return fx + (2.0f * stick - 1.0f) * (fmaxf(maxForce, fx) - fx);
}

// Forward tilt that gives the forward force fx at the vertical force fz: theta = atan(fx / lift), lift = 1 + fz.
// Tilting back to brake is limited to maxBrakeTilt. Forward the tilt is only limited by the thrust (bankLimitTilt()).
STATIC_UNIT_TESTED float bankForceToTilt(float fx, float fz, float maxBrakeTilt)
{
    const float lift = fmaxf(1.0f + fz, BANK_MIN_LIFT);
    return fmaxf(atanf(fx / lift), -maxBrakeTilt);
}

// A coordinated turn needs a yaw rate of g * tan(bank) / v, so when the bank is taken off, the rate the integrator
// holds is no longer needed in the same proportion. Scales the integrator by tan(bank) / tan(previous bank) when the
// bank shrinks or changes side (to 0 when it is centred), and leaves it to the ball when the bank grows.
// Without this, rolling out leaves the turn rate in the integrator, and the nose keeps turning until the ball has
// slowly unwound it. Returns the bank tangent to pass in next time.
STATIC_UNIT_TESTED float bankScaleBallIntegrator(float bankTan, float previousBankTan, float *integrator)
{
    if (fabsf(bankTan) < fabsf(previousBankTan) || bankTan * previousBankTan < 0.0f) {
        *integrator *= fmaxf(bankTan / previousBankTan, 0.0f);
    }
    return bankTan;
}

// Yaw rate of a coordinated turn, deg/s, positive counter-clockwise. omega = g * tan(phi) / v, and with the speed in g
// of drag (v = speed / k) that is turnGain * tan(phi) / speed, where turnGain = g * k is a constant of the craft. So the
// nose is turned at the rate the bank needs straight away, instead of waiting for the craft to slip and the ball to
// catch up. Below BANK_TURN_MIN_SPEED it fades to 0, so a bank from the hover does not spin the nose. Reversing,
// the speed is negative and the turn changes direction, like a car's.
STATIC_UNIT_TESTED float bankTurnFeedforward(float bankTan, float speed, float turnGain)
{
    return -turnGain * bankTan * speed / fmaxf(speed * speed, sq(BANK_TURN_MIN_SPEED));
}

// The steepest bank whose coordinated turn the yaw can follow at this speed: omega = turnGain * tan(phi) / speed must stay
// within BANK_TURN_RATE_SHARE of the yaw rate limit. Slow, a steep bank asks for a turn far tighter than the nose can
// be yawed into (an 80 degree bank at 5 m/s is a turn of a metre and over 1000 deg/s), and it becomes a sideways rocket
// instead: the craft slides out, accelerates, and the yaw that cannot keep up pitches the nose. Never below
// BANK_MIN_BANK_LIMIT_DEG. speed is in g of drag, turnGain in deg/s * g, maxYawRate in deg/s.
STATIC_UNIT_TESTED float bankCoordinatedBankLimit(float speed, float turnGain, float maxYawRate)
{
    const float tanMax = BANK_TURN_RATE_SHARE * maxYawRate * fabsf(speed) / fmaxf(turnGain, 1.0f);
    return fmaxf(atanf(tanMax), DEGREES_TO_RADIANS(BANK_MIN_BANK_LIMIT_DEG));
}

// Learns the turn gain in a steady banked turn: the yaw rate the ball integrator still has to hold is a turn gain error,
// so it is moved into the gain bit by bit, with the total yaw rate unchanged at that instant. Returns the new gain.
STATIC_UNIT_TESTED float bankLearnTurnGain(float turnGain, float bankTan, float speed, float dt, float *integrator)
{
    if (fabsf(bankTan) < BANK_TURN_LEARN_MIN_TAN || speed < BANK_TURN_MIN_SPEED) {
        return turnGain;
    }
    const float moved = *integrator * dt / (BANK_TURN_LEARN_TAU_S + dt);
    const float newGain = constrainf(turnGain - moved * speed / bankTan,
        BANK_TURN_GAIN_PRIOR / BANK_TURN_GAIN_RANGE, BANK_TURN_GAIN_PRIOR * BANK_TURN_GAIN_RANGE);
    // only what the gain took is taken out of the integrator, so a clamped gain leaves the rest to the ball
    *integrator -= (turnGain - newGain) * bankTan / speed;
    return newGain;
}

// PI controller on the ball. The proportional part damps, the integral part holds the yaw rate a steady
// turn needs, so the ball settles exactly at the centre. Returns the yaw rate in degrees per second.
STATIC_UNIT_TESTED float bankStepBallController(float ball, float kp, float ki, float maxRate, float dt, float *integrator)
{
    *integrator = constrainf(*integrator + ki * ball * dt, -maxRate, maxRate);
    return constrainf(kp * ball + *integrator, -maxRate, maxRate);
}

STATIC_UNIT_TESTED void bankCompute(const bankModeConfig_t *config, const bankInputs_t *in, bankState_t *state, bankOutputs_t *out)
{
    const float deadband = config->stickDeadband / 100.0f;

    const float thrustToWeight = config->thrustToWeight / 100.0f;
    if (state->turnGain == 0.0f) {
        state->turnGain = BANK_TURN_GAIN_PRIOR;
    }
    if (state->thrustTrim == 0.0f) {
        state->thrustTrim = 1.0f;
    }
    const float measuredTiltCos = cosf(DEGREES_TO_RADIANS(in->measuredPitchDeg)) * cosf(DEGREES_TO_RADIANS(in->measuredRollDeg));
    if (in->armed) {
        const float verticalForce = bankCalcVerticalForce(in->accX, in->accY, in->accZ, in->measuredPitchDeg, in->measuredRollDeg);
        const float modelVertical = bankThrustFromCollective(state->collective, thrustToWeight) * measuredTiltCos;
        if (state->trimModelVertical == 0.0f) {
            state->trimModelVertical = modelVertical;
        } else {
            state->trimModelVertical += pt1FilterGain(BANK_TRIM_LAG_HZ, in->dt) * (modelVertical - state->trimModelVertical);
        }
        // With the mixer saturated (a hard yaw, a head-check slew, a roll-in) the thrust is not collective * T/W
        state->thrustTrim = bankUpdateThrustTrim(state->thrustTrim, verticalForce, state->trimModelVertical,
            !in->motorsSaturated && measuredTiltCos >= BANK_TRIM_MIN_TILT_COS, in->dt);
    } else {
        state->thrustTrim = 1.0f;
        state->trimModelVertical = 0.0f;
    }

    float rollStick = bankShapeStick(in->roll, deadband);
    float speedStick = bankShapeStick(in->speed, deadband);     // 0..1, the deadband is at the bottom

    // Engaged with the throttle low (arming): no tilt until the bank stick has been centred once
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
    // The stick is the lift force and nothing else: at the centre it asks for no vertical acceleration, so a climb or a
    // sink carries on until the pilot stops it. There is no vertical speed estimate.
    const float fzStick = bankLiftFromStick(liftStick, thrustToWeight) - 1.0f;
    float fz = fzStick;

    // Speed stick = forward speed along the travel direction (in g of drag, full stick = the top speed).
    // The measured speed comes from the drag, levelled with the measured attitude and seen from the travel frame.
    const float maxBrakeTilt = DEGREES_TO_RADIANS(config->maxBrakeAngle);
    // The props make less thrust per collective in fast flight; the trim measures how much, so the tilt limit and the
    // top speed use it
    const float trimmedThrustToWeight = thrustToWeight * state->thrustTrim;
    const float measuredPitch = DEGREES_TO_RADIANS(in->measuredPitchDeg);
    const float measuredRoll = DEGREES_TO_RADIANS(in->measuredRollDeg);
    // The drag is filtered (BANK_DRAG_FILTER_HZ), so the thrust it is normalised by gets the same filter. Unfiltered, every
    // twitch of the lift stick would go straight into the speed and from there into the pitch and the turn rate.
    // The last update's collective was set for the trimmed thrust, so that is the thrust it makes: at 40 m/s the
    // untrimmed model says three times the real thrust, and the speed would read low and the turns too tight.
    const float thrust = bankThrustFromCollective(state->collective, thrustToWeight * state->thrustTrim);
    if (state->dragThrust == 0.0f || thrust <= 0.0f) {
        state->dragThrust = thrust;
    } else {
        state->dragThrust += pt1FilterGain(BANK_DRAG_FILTER_HZ, in->dt) * (thrust - state->dragThrust);
    }
    const float dragScale = bankDragThrustScale(state->dragThrust);
    float bodyDragX = in->dragX * dragScale;
    float bodyDragY = in->dragY * dragScale;
    float levelDragX;
    float levelDragY;
    bankDragToLevelFrame(bodyDragX, bodyDragY, measuredPitch, measuredRoll, &levelDragX, &levelDragY);
    float forwardDrag;
    float lateralDrag;
    bankDragToTravelFrame(levelDragX, levelDragY, state->headCheckOffset, &forwardDrag, &lateralDrag);
    // forward speed in g of drag, positive forwards (the drag points backwards)
    const float speed = fapplyDeadband(-forwardDrag, config->speedDeadband / 1000.0f);
    const float midSpeed = config->midSpeed / 100.0f;
    const float targetSpeed = bankTargetSpeed(speedStick, midSpeed);
    const float forwardForce = bankSpeedForce(speedStick, speed, midSpeed, config->speedKp / 100.0f,
        bankMaxForwardForce(fz, trimmedThrustToWeight));
    const float coordinatedBank = bankCoordinatedBankLimit(speed, state->turnGain, config->yawMaxRate);
    phi = constrainf(phi, -coordinatedBank, coordinatedBank);
    float theta = bankForceToTilt(forwardForce, fz, maxBrakeTilt);

    fz = bankLimitTilt(fz, &theta, &phi, trimmedThrustToWeight);
    // The turn is the bank in the travel frame: the head check rotates the tilt into the body frame below, and a
    // forward tilt then shows up as body roll on a straight track
    const float bankTan = tanf(phi);
    state->ballBankTan = bankScaleBallIntegrator(bankTan, state->ballBankTan, &state->ballIntegrator);
    const float collective = bankCalcCollective(fz, bankCollectiveTiltCos(measuredTiltCos, fz, trimmedThrustToWeight),
        trimmedThrustToWeight);
    state->collective = collective;

    const float previousOffset = state->headCheckOffset;
    const float offsetTarget = yawStick * DEGREES_TO_RADIANS(config->headCheckRange);
    state->headCheckOffset = bankSlewHeadCheck(previousOffset, offsetTarget, DEGREES_TO_RADIANS(config->headCheckRate), in->dt);
    const bool headCheckSlewing = state->headCheckOffset != previousOffset;
    const float offsetRateDps = in->dt > 0.0f ? RADIANS_TO_DEGREES((state->headCheckOffset - previousOffset) / in->dt) : 0.0f;

    bankRotateTilt(&theta, &phi, state->headCheckOffset);

    const float maxYawRate = config->yawMaxRate;
    float coordinationRateDps = 0.0f;
    float ball = 0.0f;
    if (liftStick == 0.0f) {
        // no lift: on the ground or falling, there is nothing to turn
        state->ballIntegrator = 0.0f;
    } else {
        ball = bankCalcBall(levelDragX, levelDragY, state->headCheckOffset, config->ballDeadband / 1000.0f);
        // While the head check slews, the ball is measured from where the nose is going to be, so it mostly shows
        // the yaw loop's lag behind the offset, not a slip. Integrating or learning from that would leave a yaw rate
        // behind that turns the nose away from the offset once the slew stops, and the ball deadband would hold it there.
        float ballKi = 0.0f;
        if (!headCheckSlewing) {
            ballKi = config->ballKi;
            state->turnGain = bankLearnTurnGain(state->turnGain, bankTan, speed, in->dt, &state->ballIntegrator);
        }
        coordinationRateDps = bankTurnFeedforward(bankTan, speed, state->turnGain)
            + bankStepBallController(ball, config->ballKp, ballKi, maxYawRate, in->dt, &state->ballIntegrator);
    }

    out->pitchDeg = RADIANS_TO_DEGREES(theta);
    out->rollDeg = RADIANS_TO_DEGREES(phi);
    out->yawRateDps = constrainf(coordinationRateDps + offsetRateDps, -maxYawRate, maxYawRate);
    out->collective = collective;   // no lift gives zero by itself

    DEBUG_SET(DEBUG_BANK, 0, lrintf(speed * 1000.0f));                       //!< Speed Estimate [unit:0.001g]
    DEBUG_SET(DEBUG_BANK, 1, lrintf(targetSpeed * 1000.0f));                 //!< Target Speed [unit:0.001g]
    DEBUG_SET(DEBUG_BANK, 2, lrintf(state->thrustTrim * 1000.0f));           //!< Thrust Trim [unit:0.001]
    DEBUG_SET(DEBUG_BANK, 3, lrintf(forwardForce * 1000.0f));                //!< Forward Force Command [unit:0.001g]
    DEBUG_SET(DEBUG_BANK, 4, lrintf(thrustToWeight * 100.0f));               //!< Thrust To Weight [unit:0.01]
    DEBUG_SET(DEBUG_BANK, 5, lrintf(state->turnGain * 10.0f));               //!< Turn Gain [unit:0.1deg/s]
    DEBUG_SET(DEBUG_BANK, 6, lrintf(ball * 1000.0f));                        //!< Ball [unit:0.001g]
    DEBUG_SET(DEBUG_BANK, 7, lrintf(fz * 1000.0f));                          //!< Vertical Force Command [unit:0.001g]
}

// Everything starts again when the mode is switched on, apart from the learnt turn gain. With the collective back at 0
// there is nothing to learn from until the mode has set one again.
static void bankReset(void)
{
    bankState = (bankState_t){ .turnGain = bankState.turnGain };
    bankOutputs = (bankOutputs_t){ 0 };
}

void bankModeUpdate(timeUs_t currentTimeUs)
{
    // The learnt turn gain survives the mode being switched off and on, but not a disarm (new battery, new config)
    if (!ARMING_FLAG(ARMED)) {
        bankState.turnGain = 0.0f;
    }

    if (!FLIGHT_MODE(BANK_MODE)) {
        bankReset();
        return;
    }

    float dt = BANK_FALLBACK_DT_S;
    bool restartFilters = bankState.lastUpdateUs == 0;
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

    const float accX = acc.accADC.x * acc.dev.acc_1G_rec;
    const float accY = acc.accADC.y * acc.dev.acc_1G_rec;
    if (restartFilters) {
        bankState.dragX = accX;
        bankState.dragY = accY;
    } else {
        const float gain = pt1FilterGain(BANK_DRAG_FILTER_HZ, dt);
        bankState.dragX += gain * (accX - bankState.dragX);
        bankState.dragY += gain * (accY - bankState.dragY);
    }

    const bankInputs_t inputs = {
        // The L/R sticks are swapped: yaw stick = bank, roll stick = head check. Both are positive for
        // the stick to the right, and the yaw deflection is positive to the left, hence the signs.
        .roll = -getRcDeflection(FD_YAW),
        .speed = (rcCommand[THROTTLE] - PWM_RANGE_MIN) / (float)PWM_RANGE,   // 0 at the bottom, 1 at the top
        .yaw = -getRcDeflection(FD_ROLL),
        .lift = 0.5f * (getRcDeflection(FD_PITCH) + 1.0f),
        .throttleLow = calculateThrottleStatus() == THROTTLE_LOW,
        .armed = ARMING_FLAG(ARMED),
        .accX = accX,
        .accY = accY,
        .accZ = acc.accADC.z * acc.dev.acc_1G_rec,
        .dragX = bankState.dragX,
        .dragY = bankState.dragY,
        .measuredPitchDeg = attitude.raw[FD_PITCH] * 0.1f,
        .measuredRollDeg = attitude.raw[FD_ROLL] * 0.1f,
        .dt = dt,
        // the mixer could not give the collective it was asked for last time, so the thrust measured is not its
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

#endif // USE_BANK_MODE
