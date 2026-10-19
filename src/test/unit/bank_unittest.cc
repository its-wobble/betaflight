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

#include <stdint.h>

#include <cmath>
#include <cstring>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"
    #include "common/axis.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"
    #include "flight/bank_mode.h"
    #include "flight/imu.h"
    #include "io/beeper.h"
    #include "pg/bank_mode.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"
    #include "rx/rx.h"
    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"

    extern uint8_t __config_start;
    extern uint8_t __config_end;

    acc_t acc;
    gyro_t gyro;
    attitudeEulerAngles_t attitude;
    float rcCommand[4];
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);

    float testDeflection[3];
    throttleStatus_e testThrottleStatus = THROTTLE_HIGH;
    int16_t debug[DEBUG16_VALUE_COUNT];
    float testMotorMixRange = 0.0f;
    float getMotorMixRange(void) { return testMotorMixRange; }
    uint8_t debugMode;

    void beeperConfirmationBeeps(uint8_t) {}
    float getRcDeflection(int axis) { return testDeflection[axis]; }
    throttleStatus_e calculateThrottleStatus(void) { return testThrottleStatus; }

    float bankShapeStick(float deflection);
    float bankLiftFromStick(float stick, float thrustToWeight);
    float bankThrustFromCollective(float collective, float thrustToWeight);
    float bankCollectiveFromThrust(float thrust, float thrustToWeight);
    void bankRotate(float *forward, float *left, float angle);
    void bankBodyToHeading(float x, float y, float z, float roll, float pitch, float *forward, float *left, float *up);
    void bankAnglesFromThrust(float forward, float right, float up, float *rollDeg, float *pitchDeg);
    void bankLevelVelocity(float x, float y, float roll, float pitch, float *forward, float *left);
    void bankHeadingToBody(float forward, float left, float up, float roll, float pitch, float *x, float *y, float *z);
    float bankShapeSpeedStick(float stick);
    float bankTargetSpeed(float speedStick, float midSpeed);
    float bankSpeedLoop(float targetSpeed, float v);
    float bankSpeedForce(float speedStick, float v, float midSpeed, float maxForce);
    float bankCornerSpeed(float force, float dragRate, float turnRate);
    float bankCornerShare(float bankStick);
    void bankThrustVector(float lift, float thrust, float bankAngle, float cornerForce, float forwardForce, float *forward, float *right, float *up);
    float bankHeadingRate(float gyroPitch, float gyroYaw, float roll, float pitch);
    float bankUpdateThrustTrim(float trim, float measuredThrust, float modelThrust, bool mayBeOnGround, float dt);
    float bankLeakThrustTrim(float trim, float dt);
    float bankScaleTrimWithInflow(float trim, float inflow, float previousInflow);
    void bankCompute(const bankModeConfig_t *config, const bankInputs_t *in, bankState_t *state, bankOutputs_t *out);
}

#include "gtest/gtest.h"

static const float DEG = M_PI / 180.0f;

static float thrustToWeight(void)
{
    return bankModeConfig()->thrustToWeight / 100.0f;
}

static float dragRate(void)
{
    return bankModeConfig()->dragRate / 100.0f;
}

static float hoverCollective(void)
{
    return bankCollectiveFromThrust(1.0f, thrustToWeight());
}

// level, sticks centred, Speed at the bottom, armed, at rest
static bankInputs_t centredInputs(void)
{
    bankInputs_t in = {};
    in.lift = 0.5f;
    in.accZ = 1.0f;
    in.dt = 0.01f;
    in.armed = true;
    return in;
}

static void computeN(bankInputs_t *in, bankState_t *state, bankOutputs_t *out, int n)
{
    for (int i = 0; i < n; i++) {
        bankCompute(bankModeConfig(), in, state, out);
    }
}

// The thrust direction the angle targets ask for, in the heading frame.
static void targetThrust(const bankOutputs_t *out, float *forward, float *left, float *up)
{
    bankBodyToHeading(0.0f, 0.0f, 1.0f, out->rollDeg * DEG, out->pitchDeg * DEG, forward, left, up);
}

TEST(BankModeTest, SticksHaveADeadbandAndStillReachFullDeflection)
{
    EXPECT_FLOAT_EQ(0.0f, bankShapeStick(0.04f));
    EXPECT_FLOAT_EQ(1.0f, bankShapeStick(1.0f));
    EXPECT_FLOAT_EQ(-1.0f, bankShapeStick(-1.0f));
    EXPECT_NEAR(0.5f, bankShapeStick(0.525f), 1e-5f);
}

TEST(BankModeTest, LiftIsALittleAtTheBottomOneAtTheCentreAndAllTheThrustAtTheTop)
{
    EXPECT_GT(bankLiftFromStick(0.0f, 5.7f), 0.0f);
    EXPECT_LT(bankLiftFromStick(0.0f, 5.7f), 0.5f);
    EXPECT_FLOAT_EQ(1.0f, bankLiftFromStick(0.5f, 5.7f));
    EXPECT_FLOAT_EQ(5.7f, bankLiftFromStick(1.0f, 5.7f));
    EXPECT_NEAR(bankLiftFromStick(0.5f, 5.7f) - bankLiftFromStick(0.49f, 5.7f), bankLiftFromStick(0.51f, 5.7f) - bankLiftFromStick(0.5f, 5.7f), 1e-4f);
    EXPECT_FLOAT_EQ(1.5f, bankLiftFromStick(1.0f, 1.5f));
    for (float s = 0.0f; s < 1.0f; s += 0.01f) {
        EXPECT_LT(bankLiftFromStick(s, 5.7f), bankLiftFromStick(s + 0.01f, 5.7f));
        EXPECT_LT(bankLiftFromStick(s, 1.9f), bankLiftFromStick(s + 0.01f, 1.9f));
    }
}

TEST(BankModeTest, CollectiveIsTheInverseOfTheThrustCurve)
{
    for (float c = 0.0f; c <= 1.0f; c += 0.1f) {
        EXPECT_NEAR(c, bankCollectiveFromThrust(bankThrustFromCollective(c, 5.7f), 5.7f), 1e-5f);
    }
    EXPECT_FLOAT_EQ(1.0f, bankCollectiveFromThrust(10.0f, 5.7f));
}

TEST(BankModeTest, RotatingIsCounterClockwiseAndBackIsTheIdentity)
{
    float forward = 1.0f;
    float left = 0.0f;
    bankRotate(&forward, &left, 90.0f * DEG);
    EXPECT_NEAR(0.0f, forward, 1e-6f);
    EXPECT_NEAR(1.0f, left, 1e-6f);

    forward = 0.3f;
    left = -0.7f;
    bankRotate(&forward, &left, 0.6f);
    bankRotate(&forward, &left, -0.6f);
    EXPECT_NEAR(0.3f, forward, 1e-6f);
    EXPECT_NEAR(-0.7f, left, 1e-6f);
}

TEST(BankModeTest, TheThrustPointsWhereTheAnglesSay)
{
    float forward;
    float left;
    float up;
    // pitched nose-down: forward; rolled right: to the right
    bankBodyToHeading(0.0f, 0.0f, 1.0f, 0.0f, 30.0f * DEG, &forward, &left, &up);
    EXPECT_NEAR(0.5f, forward, 1e-5f);
    EXPECT_NEAR(0.0f, left, 1e-5f);
    bankBodyToHeading(0.0f, 0.0f, 1.0f, 30.0f * DEG, 0.0f, &forward, &left, &up);
    EXPECT_NEAR(-0.5f, left, 1e-5f);

    // the angle targets for a thrust direction point the thrust there, combined too
    const float cases[][3] = { { 0.3f, 0.0f, 1.0f }, { 0.0f, 2.0f, 1.0f }, { 1.5f, -2.0f, 0.5f }, { -0.5f, 3.0f, 1.0f } };
    for (const auto &c : cases) {
        float rollDeg;
        float pitchDeg;
        bankAnglesFromThrust(c[0], c[1], c[2], &rollDeg, &pitchDeg);
        bankBodyToHeading(0.0f, 0.0f, 1.0f, rollDeg * DEG, pitchDeg * DEG, &forward, &left, &up);
        const float length = sqrtf(sq(c[0]) + sq(c[1]) + sq(c[2]));
        EXPECT_NEAR(c[0] / length, forward, 1e-5f);
        EXPECT_NEAR(-c[1] / length, left, 1e-5f);
        EXPECT_NEAR(c[2] / length, up, 1e-5f);
    }
}

TEST(BankModeTest, HeadingToBodyIsTheTranspose)
{
    const float roll = 40.0f * DEG;
    const float pitch = -25.0f * DEG;
    float x;
    float y;
    float z;
    bankHeadingToBody(0.3f, -0.2f, 0.9f, roll, pitch, &x, &y, &z);
    float forward;
    float left;
    float up;
    bankBodyToHeading(x, y, z, roll, pitch, &forward, &left, &up);
    EXPECT_NEAR(0.3f, forward, 1e-5f);
    EXPECT_NEAR(-0.2f, left, 1e-5f);
    EXPECT_NEAR(0.9f, up, 1e-5f);
}

TEST(BankModeTest, LevelTravelIsReadBackFromTheRotorPlane)
{
    const float roll = 20.0f * DEG;
    const float pitch = 30.0f * DEG;
    float x;
    float y;
    float z;
    bankHeadingToBody(0.3f, -0.1f, 0.0f, roll, pitch, &x, &y, &z);
    float forward;
    float left;
    bankLevelVelocity(x, y, roll, pitch, &forward, &left);
    EXPECT_NEAR(0.3f, forward, 1e-5f);
    EXPECT_NEAR(-0.1f, left, 1e-5f);
}

TEST(BankModeTest, SpeedStickIsASpeedInTheLowerHalfAndAForceInTheUpperHalf)
{
    // at the target the speed loop gives the drag of that speed
    EXPECT_FLOAT_EQ(0.5f, bankSpeedForce(0.5f, 0.5f, 0.5f, 3.0f));
    // the bottom third brakes a craft that is too fast, above it the craft coasts
    EXPECT_LT(bankSpeedForce(0.2f, 0.8f, 0.5f, 3.0f), 0.0f);
    EXPECT_FLOAT_EQ(0.0f, bankSpeedForce(0.4f, 0.8f, 0.5f, 3.0f));
    EXPECT_GT(bankSpeedForce(0.4f, 0.41f, 0.5f, 3.0f), 0.0f);
    // the top of the stick is all the force there is, continuous at the centre
    EXPECT_FLOAT_EQ(3.0f, bankSpeedForce(1.0f, 0.2f, 0.5f, 3.0f));
    EXPECT_NEAR(bankSpeedForce(0.5f, 0.2f, 0.5f, 3.0f), bankSpeedForce(0.5001f, 0.2f, 0.5f, 3.0f), 1e-3f);
}

TEST(BankModeTest, SpeedStickHasAStopBandAndKeepsItsCentre)
{
    EXPECT_FLOAT_EQ(0.0f, bankShapeSpeedStick(0.1f));
    EXPECT_FLOAT_EQ(0.5f, bankShapeSpeedStick(0.5f));
    EXPECT_FLOAT_EQ(0.8f, bankShapeSpeedStick(0.8f));
    EXPECT_FLOAT_EQ(-0.05f * 2.0f * 0.5f, bankTargetSpeed(-0.05f, 0.5f));
}

TEST(BankModeTest, CornerSpeedIsWhereTheForceTurnsTheTrackAtTheCornerRate)
{
    // omega = dragRate * force / speed
    const float v = bankCornerSpeed(5.0f, 0.4f, 300.0f * DEG);
    EXPECT_NEAR(300.0f * DEG, 0.4f * 5.0f / v, 1e-4f);
}

TEST(BankModeTest, TheCornerSpeedTakesOverNearFullBank)
{
    EXPECT_FLOAT_EQ(0.0f, bankCornerShare(0.5f));
    EXPECT_NEAR(0.5f, bankCornerShare(0.725f), 1e-5f);
    EXPECT_FLOAT_EQ(1.0f, bankCornerShare(0.9f));
}

TEST(BankModeTest, BankKeepsItsAngleWhileLiftGivesWayAFifth)
{
    float forward;
    float right;
    float up;
    // a 45 degree bank at 1 g within the thrust: Lift held, the forward force gets what is left
    bankThrustVector(1.0f, 4.0f, 45.0f * DEG, 0.0f, 10.0f, &forward, &right, &up);
    EXPECT_NEAR(1.0f, up, 1e-5f);
    EXPECT_NEAR(1.0f, right, 1e-5f);
    EXPECT_NEAR(sqrtf(16.0f - 2.0f), forward, 1e-4f);

    // past the thrust Lift gives way at the bank angle, down to 0.8 of it, then the angle gives way
    bankThrustVector(1.0f, 2.0f, -60.0f * DEG, 0.0f, 10.0f, &forward, &right, &up);
    EXPECT_NEAR(1.0f, up, 1e-4f);
    EXPECT_NEAR(-sqrtf(3.0f), right, 1e-4f);
    bankThrustVector(1.0f, 4.0f, -85.0f * DEG, 0.0f, 10.0f, &forward, &right, &up);
    EXPECT_NEAR(0.8f, up, 1e-4f);
    EXPECT_NEAR(-sqrtf(16.0f - 0.64f), right, 1e-4f);
    EXPECT_NEAR(0.0f, forward, 1e-3f);

    // a craft with thrust to spare keeps the bank angle and all of Lift
    bankThrustVector(1.0f, 20.0f, 85.0f * DEG, 0.0f, 0.0f, &forward, &right, &up);
    EXPECT_NEAR(85.0f, atan2f(right, up) / DEG, 1e-3f);
    EXPECT_NEAR(1.0f, up, 1e-4f);
}

TEST(BankModeTest, TheCornerForceComesBeforeBank)
{
    float forward;
    float right;
    float up;
    // a brake towards the corner speed takes its share of the thrust first
    bankThrustVector(1.0f, 4.0f, 85.0f * DEG, -1.0f, 0.0f, &forward, &right, &up);
    EXPECT_NEAR(-1.0f, forward, 1e-4f);
    EXPECT_NEAR(sqrtf(15.0f - sq(up)), right, 1e-4f);
    // and so does the force that holds the corner speed of a slow craft
    bankThrustVector(1.0f, 4.0f, 85.0f * DEG, 0.5f, 10.0f, &forward, &right, &up);
    EXPECT_NEAR(0.5f, forward, 1e-3f);
    // no further back than the brake angle
    bankThrustVector(1.0f, 4.0f, 0.0f, -3.0f, 0.0f, &forward, &right, &up);
    EXPECT_NEAR(-tanf(60.0f * DEG), forward, 1e-4f);
}

TEST(BankModeTest, ABrakeAfterBankTakesWhatIsLeftWithinItsAngle)
{
    float forward;
    float right;
    float up;
    bankThrustVector(1.0f, 5.0f, 0.0f, 0.0f, -10.0f, &forward, &right, &up);
    EXPECT_NEAR(-tanf(60.0f * DEG), forward, 1e-4f);
    bankThrustVector(1.0f, 2.0f, 45.0f * DEG, 0.0f, -10.0f, &forward, &right, &up);
    EXPECT_NEAR(-sqrtf(4.0f - 2.0f), forward, 1e-4f);
}

TEST(BankModeTest, HeadingRateIsTheYawRateLevelAndSharedOutBanked)
{
    EXPECT_FLOAT_EQ(1.0f, bankHeadingRate(0.0f, 1.0f, 0.0f, 0.0f));
    // a coordinated turn right at a 60 degree bank: the turn is seen on pitch and yaw
    const float omega = -1.0f;
    const float roll = 60.0f * DEG;
    EXPECT_NEAR(omega, bankHeadingRate(omega * sinf(roll), omega * cosf(roll), roll, 0.0f), 1e-5f);
}

TEST(BankModeTest, ThrustTrimFollowsTheMeasuredThrustButNotDownOnTheGround)
{
    float trim = 1.0f;
    for (int i = 0; i < 200; i++) {
        trim = bankUpdateThrustTrim(trim, 0.8f, 1.0f, false, 0.01f);
    }
    EXPECT_NEAR(0.8f, trim, 1e-3f);
    // missing thrust is added off the ground
    trim = 1.0f;
    for (int i = 0; i < 200; i++) {
        trim = bankUpdateThrustTrim(trim, 0.5f, 1.0f, false, 0.01f);
    }
    EXPECT_NEAR(0.5f, trim, 1e-3f);
    // on the ground, which hides missing thrust: no lower than 1
    trim = 1.0f;
    for (int i = 0; i < 200; i++) {
        trim = bankUpdateThrustTrim(trim, 1.0f, 1.5f, true, 0.01f);
    }
    EXPECT_FLOAT_EQ(1.0f, trim);
}

TEST(BankModeTest, OnlyATrimThatAddsThrustLeaksBack)
{
    EXPECT_LT(bankLeakThrustTrim(1.5f, 0.1f), 1.5f);
    EXPECT_FLOAT_EQ(0.7f, bankLeakThrustTrim(0.7f, 0.1f));
}

TEST(BankModeTest, ThrustTrimAddsLessAtOnceWhenTheInflowFalls)
{
    EXPECT_FLOAT_EQ(0.6f, bankScaleTrimWithInflow(0.6f, 0.5f, 0.4f));
    EXPECT_GT(bankScaleTrimWithInflow(0.6f, 0.0f, 0.4f), 0.6f);
}

TEST(BankModeTest, CentredSticksHoverLevel)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = false;       // no trim
    computeN(&in, &state, &out, 100);
    EXPECT_NEAR(0.0f, out.rollDeg, 1e-4f);
    EXPECT_NEAR(0.0f, out.pitchDeg, 1e-4f);
    EXPECT_NEAR(0.0f, out.yawRateDps, 1e-4f);
    EXPECT_NEAR(hoverCollective(), out.collective, 1e-4f);
}

TEST(BankModeTest, LiftIsKeptAtTheMeasuredTiltAndNotPastTheThrust)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = false;
    in.measuredRollDeg = 60.0f;
    computeN(&in, &state, &out, 1);
    EXPECT_NEAR(2.0f, bankThrustFromCollective(out.collective, thrustToWeight()), 1e-3f);
    in.measuredRollDeg = 89.0f;
    computeN(&in, &state, &out, 1);
    EXPECT_NEAR(0.95f * thrustToWeight(), bankThrustFromCollective(out.collective, thrustToWeight()), 1e-3f);
    in.measuredRollDeg = 120.0f;
    computeN(&in, &state, &out, 1);
    EXPECT_FLOAT_EQ(0.0f, out.collective);
}

TEST(BankModeTest, TheVelocityIntegratesTheForceAndFollowsTheDrag)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    computeN(&in, &state, &out, 1);
    // a forward specific force with no drag yet: the velocity grows at dragRate * force at first
    in.measuredPitchDeg = 30.0f;
    in.accZ = 1.0f / cosf(30.0f * DEG);
    computeN(&in, &state, &out, 10);
    const float force = in.accZ * sinf(30.0f * DEG);
    EXPECT_NEAR(dragRate() * force * 0.1f, state.velForward, 0.3f * dragRate() * force * 0.1f);

    // steady flight, the forward tilt balancing the drag, reads 1 g straight up: the rotor plane sees the drag of
    // tan(pitch), and the velocity settles on it
    state = {};
    in = centredInputs();
    in.armed = false;
    const float pitch = 20.0f * DEG;
    in.measuredPitchDeg = 20.0f;
    in.accX = -sinf(pitch);
    in.accZ = cosf(pitch);
    in.dt = 0.0f;           // engaged: the drag is read as level travel at once
    computeN(&in, &state, &out, 1);
    in.dt = 0.01f;
    computeN(&in, &state, &out, 500);
    const float dragScale = 1.0f / sqrtf(bankThrustFromCollective(state.collective, thrustToWeight()));
    EXPECT_NEAR(tanf(pitch) * dragScale, state.velForward, 0.01f);
    EXPECT_NEAR(0.0f, state.velUp, 0.01f);
}

TEST(BankModeTest, AClimbAcrossTiltedRotorsIsNotReadAsSlip)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = false;
    computeN(&in, &state, &out, 1);
    // banked 45 degrees and climbing at 0.1 g of drag: the climb's drag shows on body Y as if drifting sideways,
    // which a level reading of the drag would take for 0.1 g of drag of slip
    state = {};
    state.velUp = 0.1f;
    state.lastUpdateUs = 1;
    in.measuredRollDeg = 45.0f;
    float x;
    float y;
    float z;
    bankHeadingToBody(0.0f, 0.0f, 0.1f, 45.0f * DEG, 0.0f, &x, &y, &z);
    in.accX = -x;
    in.accY = -y;
    in.accZ = 1.0f / cosf(45.0f * DEG);
    bankCompute(bankModeConfig(), &in, &state, &out);
    // the drag is what the climb makes, so nothing is corrected: only the sideways force is integrated
    float forward;
    float left;
    float up;
    bankBodyToHeading(in.accX, in.accY, in.accZ, 45.0f * DEG, 0.0f, &forward, &left, &up);
    EXPECT_NEAR(dragRate() * left * in.dt, state.velLeft, 1e-4f);
}

TEST(BankModeTest, TheVelocityTurnsBackAsTheNoseTurns)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    computeN(&in, &state, &out, 1);
    state.velForward = 0.5f;
    // the nose yaws 90 degrees left in half a second, no force: the velocity is now to the right
    in.gyroYawDps = 180.0f;
    in.armed = false;
    bankState_t probe = state;
    for (int i = 0; i < 50; i++) {
        probe.collective = 0.0f;
        bankCompute(bankModeConfig(), &in, &probe, &out);
    }
    const float magnitude = sqrtf(sq(probe.velForward) + sq(probe.velLeft));
    EXPECT_LT(probe.velLeft, -0.5f * magnitude);
}

TEST(BankModeTest, BankRightRollsRightAndSpeedPitchesForward)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = false;
    in.accX = -0.4f;
    in.speed = 0.75f;
    in.bank = 0.5f;
    computeN(&in, &state, &out, 300);
    float forward;
    float left;
    float up;
    targetThrust(&out, &forward, &left, &up);
    // half the stick is half the bank angle limit
    EXPECT_NEAR(0.5f * 85.0f * (0.5f - 0.05f) / 0.95f / 0.5f, atan2f(-left, up) / DEG, 0.5f);
    EXPECT_GT(out.rollDeg, 10.0f);
    EXPECT_GT(out.pitchDeg, 5.0f);
}

TEST(BankModeTest, FullBankTurnsWithNearlyAllTheThrust)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = false;
    in.accX = -0.4f;
    in.speed = 0.5f;
    in.bank = 1.0f;
    computeN(&in, &state, &out, 300);
    float forward;
    float left;
    float up;
    targetThrust(&out, &forward, &left, &up);
    // steeper than Lift alone would allow, Lift giving way a fifth
    const float available = 0.95f * thrustToWeight();
    EXPECT_GT(atan2f(-left, up) / DEG, acosf(1.0f / available) / DEG);
    EXPECT_LT(atan2f(-left, up) / DEG, 85.0f);
    // held at that tilt the collective is nearly all the thrust there is
    in.measuredRollDeg = out.rollDeg;
    in.measuredPitchDeg = out.pitchDeg;
    computeN(&in, &state, &out, 1);
    EXPECT_GT(bankThrustFromCollective(out.collective, thrustToWeight()), 0.9f * available);
}

TEST(BankModeTest, TheThrustIsTurnedTowardsTheVectorAtUpToTheTiltRate)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = false;
    // level and a degree off: 20 deg/s per degree, back towards level
    in.measuredRollDeg = 1.0f;
    computeN(&in, &state, &out, 1);
    EXPECT_NEAR(-20.0f, out.rateDps[FD_ROLL], 0.1f);
    EXPECT_NEAR(0.0f, out.rateDps[FD_PITCH], 0.1f);
    EXPECT_NEAR(0.0f, out.rateDps[FD_YAW], 0.1f);

    // half Bank at speed, a 40 degree bank: as fast as the tilt rate allows, to the right
    in.measuredRollDeg = 0.0f;
    in.accX = -0.4f;
    computeN(&in, &state, &out, 300);
    in.bank = 0.5f;
    computeN(&in, &state, &out, 1);
    EXPECT_NEAR(900.0f, sqrtf(sq(out.rateDps[FD_ROLL]) + sq(out.rateDps[FD_PITCH])), 1.0f);
    EXPECT_GT(out.rateDps[FD_ROLL], 500.0f);
}

TEST(BankModeTest, TheHeadingsTurnIsSharedOutOverTheBodyAxes)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.bank = 1.0f;
    computeN(&in, &state, &out, 1);
    // level, at zero Speed: Bank yaws on the spot
    EXPECT_NEAR(-400.0f, out.rateDps[FD_YAW], 0.1f);
    EXPECT_NEAR(0.0f, out.rateDps[FD_ROLL], 0.1f);
    // banked and pitched on the target, the same turn about the world's up is seen on all three
    in.bank = 0.0f;
    computeN(&in, &state, &out, 1);
    const float turn = 100.0f;
    const float roll = 50.0f * DEG;
    const float pitch = 20.0f * DEG;
    float x;
    float y;
    float z;
    bankHeadingToBody(0.0f, 0.0f, turn, roll, pitch, &x, &y, &z);
    EXPECT_NEAR(-sinf(pitch) * turn, x, 1e-3f);
    EXPECT_NEAR(cosf(pitch) * sinf(roll) * turn, y, 1e-3f);
    EXPECT_NEAR(cosf(pitch) * cosf(roll) * turn, z, 1e-3f);
}

TEST(BankModeTest, ABankedTurnFeedsTheYawForwardFromTheBankTheCraftIsIn)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = false;
    in.speed = 0.5f;
    in.bank = 0.5f;
    in.accX = -0.4f;
    computeN(&in, &state, &out, 300);
    EXPECT_NEAR(0.0f, out.yawRateDps, 1.0f);   // still level: no turn yet
    in.measuredRollDeg = 45.0f;
    computeN(&in, &state, &out, 1);
    // a right turn: clockwise, at dragRate * force / speed
    const float expected = -dragRate() * bankThrustFromCollective(state.collective, thrustToWeight()) * sinf(45.0f * DEG) / 0.38f / DEG;
    EXPECT_LT(out.yawRateDps, 0.5f * expected);
}

TEST(BankModeTest, ANoseOffTheTrackYawsBackAndTheForceFollowsTheTrack)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = false;
    in.speed = 1.0f;
    // the track is 20 degrees left of the nose
    in.accX = -0.4f * cosf(20.0f * DEG);
    in.accY = -0.4f * sinf(20.0f * DEG);
    computeN(&in, &state, &out, 300);
    EXPECT_GT(out.yawRateDps, 0.0f);
    // full Speed pushes along the track, not the nose
    float forward;
    float left;
    float up;
    targetThrust(&out, &forward, &left, &up);
    EXPECT_NEAR(20.0f, atan2f(left, forward) / DEG, 1.0f);
}

TEST(BankModeTest, BankAtZeroSpeedYawsOnTheSpotWithoutTilt)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.bank = 1.0f;
    computeN(&in, &state, &out, 100);
    EXPECT_NEAR(0.0f, out.rollDeg, 1e-3f);
    EXPECT_NEAR(-400.0f, out.yawRateDps, 1e-2f);
}

TEST(BankModeTest, ALateralAccelerometerBiasDoesNotYawTheHover)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.accY = 0.02f;
    computeN(&in, &state, &out, 500);
    EXPECT_NEAR(0.0f, out.yawRateDps, 1e-3f);
}

TEST(BankModeTest, FullBankAtSpeedBrakesTowardsTheCornerSpeed)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = false;
    in.speed = 0.75f;
    in.bank = 0.9f;
    in.accX = -1.0f;        // far above the corner speed
    computeN(&in, &state, &out, 300);
    EXPECT_LT(state.velForward, 1.1f);
    float forward;
    float left;
    float up;
    targetThrust(&out, &forward, &left, &up);
    EXPECT_LT(forward, -0.05f);
}

TEST(BankModeTest, LiftAtTheBottomGivesTheFloorButNothingTurns)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = false;
    in.lift = 0.0f;
    computeN(&in, &state, &out, 10);
    EXPECT_NEAR(bankLiftFromStick(0.0f, thrustToWeight()), bankThrustFromCollective(out.collective, thrustToWeight()), 1e-4f);

    in.accX = -0.3f;
    in.accY = -0.3f;
    in.bank = 1.0f;
    in.measuredRollDeg = 30.0f;
    computeN(&in, &state, &out, 100);
    EXPECT_GT(out.collective, 0.0f);
    EXPECT_FLOAT_EQ(0.0f, out.yawRateDps);
}

TEST(BankModeTest, ThrustTrimIsNotMeasuredWhileDisarmedOrSaturated)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.accZ = 1.5f;
    in.armed = false;
    computeN(&in, &state, &out, 100);
    EXPECT_FLOAT_EQ(1.0f, state.thrustTrim);

    in.armed = true;
    in.motorsSaturated = true;
    computeN(&in, &state, &out, 100);
    EXPECT_FLOAT_EQ(1.0f, state.thrustTrim);

    in.motorsSaturated = false;
    computeN(&in, &state, &out, 100);
    EXPECT_GT(state.thrustTrim, 1.4f);
    EXPECT_LT(out.collective, hoverCollective());
}

static void setUpUpdate(void)
{
    pgResetAll();
    flightModeFlags = 0;
    bankModeUpdate(0);
    memset(testDeflection, 0, sizeof(testDeflection));
    memset(&acc, 0, sizeof(acc));
    memset(&gyro, 0, sizeof(gyro));
    memset(&attitude, 0, sizeof(attitude));
    acc.dev.acc_1G_rec = 1.0f / 512.0f;
    rcCommand[THROTTLE] = 1000;     // Speed stick at the bottom
    rcData[THROTTLE] = 1000;
    rxConfigMutable()->mincheck = 1050;
    testThrottleStatus = THROTTLE_LOW;
    DISABLE_ARMING_FLAG(ARMED);
    testMotorMixRange = 0.0f;
}

TEST(BankModeTest, EngagedWithTheThrottleLowNoTiltUntilTheBankStickIsCentred)
{
    setUpUpdate();
    ENABLE_FLIGHT_MODE(BANK_MODE);
    ENABLE_ARMING_FLAG(ARMED);
    acc.accADC.z = 512.0f;
    testDeflection[FD_YAW] = -1.0f;     // yaw stick right = bank right
    timeUs_t now = 1000;
    for (int i = 0; i < 10; i++, now += 10000) {
        bankModeUpdate(now);
    }
    EXPECT_FLOAT_EQ(0.0f, bankModeGetAngleTarget(FD_ROLL));
    EXPECT_FLOAT_EQ(0.0f, bankModeGetAngleTarget(FD_PITCH));
    EXPECT_FLOAT_EQ(0.0f, bankModeGetRate(FD_YAW));

    testDeflection[FD_YAW] = 0.0f;
    bankModeUpdate(now);
    now += 10000;
    testDeflection[FD_YAW] = -1.0f;
    bankModeUpdate(now);
    EXPECT_LT(bankModeGetRate(FD_YAW), 0.0f);
}

TEST(BankModeTest, ThrottleIsLowWithLiftAtTheBottom)
{
    setUpUpdate();
    ENABLE_FLIGHT_MODE(BANK_MODE);
    bankModeUpdate(1000);
    EXPECT_FALSE(bankModeIsThrottleLow());
    testDeflection[FD_PITCH] = -0.9f;
    bankModeUpdate(11000);
    EXPECT_FALSE(bankModeIsThrottleLow());
    testDeflection[FD_PITCH] = -1.0f;
    bankModeUpdate(21000);
    EXPECT_GT(bankModeGetCollective(), 0.0f);
    EXPECT_TRUE(bankModeIsThrottleLow());
}

TEST(BankModeTest, UpdateMirrorsTheModeFlagAndClearsWhenInactive)
{
    setUpUpdate();
    bankModeUpdate(1000);
    EXPECT_FALSE(bankModeIsActive());

    ENABLE_FLIGHT_MODE(BANK_MODE);
    acc.accADC.z = 512.0f;
    bankModeUpdate(2000);
    EXPECT_TRUE(bankModeIsActive());
    EXPECT_NEAR(hoverCollective(), bankModeGetCollective(), 1e-5f);

    DISABLE_FLIGHT_MODE(BANK_MODE);
    bankModeUpdate(22000);
    EXPECT_FALSE(bankModeIsActive());
    EXPECT_FLOAT_EQ(0.0f, bankModeGetAngleTarget(FD_ROLL));
    EXPECT_FLOAT_EQ(0.0f, bankModeGetRate(FD_ROLL));
    EXPECT_FLOAT_EQ(0.0f, bankModeGetRate(FD_YAW));
    EXPECT_FLOAT_EQ(0.0f, bankModeGetCollective());
}

TEST(BankModeTest, UpdateReadsTheGyroIntoTheVelocity)
{
    setUpUpdate();
    ENABLE_FLIGHT_MODE(BANK_MODE);
    acc.accADC.x = -0.4f * 512.0f;      // flying forwards
    acc.accADC.z = 512.0f;
    timeUs_t now = 1000;
    for (int i = 0; i < 300; i++, now += 10000) {
        bankModeUpdate(now);
    }
    // yawing left with the drag still forward: the track lags to the right of the nose, the nose yaws back right
    gyro.gyroADCf[FD_YAW] = 200.0f;
    for (int i = 0; i < 5; i++, now += 10000) {
        bankModeUpdate(now);
    }
    EXPECT_LT(bankModeGetRate(FD_YAW), 0.0f);
}

TEST(BankModeTest, UpdateReadsTheReverseBelowMinCheck)
{
    setUpUpdate();
    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_FLIGHT_MODE(BANK_MODE);
    acc.accADC.z = 512.0f;
    timeUs_t now = 1000;
    bankModeUpdate(now);
    rcCommand[THROTTLE] = 1200;
    rcData[THROTTLE] = 1240;
    bankModeUpdate(now += 10000);
    rcCommand[THROTTLE] = 1000;
    rcData[THROTTLE] = 1050;
    for (int i = 0; i < 10; i++) {
        bankModeUpdate(now += 10000);
    }
    EXPECT_NEAR(0.0f, bankModeGetAngleTarget(FD_PITCH), 1e-3f);
    rcData[THROTTLE] = 1000;
    for (int i = 0; i < 10; i++) {
        bankModeUpdate(now += 10000);
    }
    EXPECT_LT(bankModeGetAngleTarget(FD_PITCH), 0.0f);
}

TEST(BankModeTest, DebugModeBankLogsTheEstimates)
{
    setUpUpdate();
    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_FLIGHT_MODE(BANK_MODE);
    debugMode = DEBUG_BANK;
    memset(debug, 0, sizeof(debug));
    acc.accADC.x = -0.3f * 512.0f;
    rcCommand[THROTTLE] = 1500;
    testThrottleStatus = THROTTLE_HIGH;
    timeUs_t now = 1000;
    for (int i = 0; i < 200; i++, now += 10000) {
        acc.accADC.z = bankThrustFromCollective(bankModeGetCollective(), thrustToWeight()) * 512.0f;
        bankModeUpdate(now);
    }
    EXPECT_GT(debug[0], 150);           // speed estimate, 0.001 g
    EXPECT_GT(debug[1], 0);             // corner speed
    debugMode = DEBUG_NONE;
}
