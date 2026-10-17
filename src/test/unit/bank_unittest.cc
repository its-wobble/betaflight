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
    #include "sensors/acceleration.h"

    extern uint8_t __config_start;
    extern uint8_t __config_end;

    acc_t acc;
    attitudeEulerAngles_t attitude;
    float rcCommand[4];

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
    void bankRotate(float *forward, float *right, float angle);
    float bankTargetSpeed(float speedStick, float midSpeed);
    float bankSpeedForce(float speedStick, float v, float midSpeed, float maxForce);
    float bankTiltThrust(float lift, float thrustToWeight);
    void bankThrustVector(float lift, float horizontal, float bankStick, float forwardForce, float *forward, float *right);
    void bankLevelDrag(float accX, float accY, float pitch, float roll, float *forward, float *left);
    float bankTravelDirection(float forwardDrag);
    float bankTurnFeedforward(float bankTan, float speed);
    float bankUpdateThrustTrim(float trim, float verticalForce, float modelVertical, float dt);
    float bankLeakThrustTrim(float trim, float dt);
    float bankScaleTrimWithInflow(float trim, float inflow, float previousInflow);
    float bankLookIntoTurnOffset(float bankStick, float speedStick);
    void bankCompute(const bankModeConfig_t *config, const bankInputs_t *in, bankState_t *state, bankOutputs_t *out);
}

#include "gtest/gtest.h"

static const float DEG = M_PI / 180.0f;

static float thrustToWeight(void)
{
    return bankModeConfig()->thrustToWeight / 100.0f;
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

// The thrust direction (forward, right, up) the angle targets ask for, in the nose frame, scaled to unit up.
static void targetThrust(const bankOutputs_t *out, float *forward, float *right)
{
    const float pitch = out->pitchDeg * DEG;
    const float roll = out->rollDeg * DEG;
    *forward = sinf(pitch) / (cosf(pitch) * cosf(roll));
    *right = tanf(roll);
}

TEST(BankModeTest, SticksHaveADeadbandAndStillReachFullDeflection)
{
    EXPECT_FLOAT_EQ(0.0f, bankShapeStick(0.04f));
    EXPECT_FLOAT_EQ(1.0f, bankShapeStick(1.0f));
    EXPECT_FLOAT_EQ(-1.0f, bankShapeStick(-1.0f));
    EXPECT_NEAR(0.5f, bankShapeStick(0.525f), 1e-5f);
}

TEST(BankModeTest, LiftIsZeroAtTheBottomOneAtTheCentreAndAllTheThrustAtTheTop)
{
    EXPECT_FLOAT_EQ(0.0f, bankLiftFromStick(0.0f, 5.7f));
    EXPECT_FLOAT_EQ(1.0f, bankLiftFromStick(0.5f, 5.7f));
    EXPECT_FLOAT_EQ(5.7f, bankLiftFromStick(1.0f, 5.7f));
    // both halves start at 1 g per half stick
    EXPECT_NEAR(bankLiftFromStick(0.5f, 5.7f) - bankLiftFromStick(0.49f, 5.7f), bankLiftFromStick(0.51f, 5.7f) - bankLiftFromStick(0.5f, 5.7f), 1e-4f);
    EXPECT_FLOAT_EQ(1.5f, bankLiftFromStick(1.0f, 1.5f));
}

TEST(BankModeTest, CollectiveIsTheInverseOfTheThrustCurve)
{
    for (float c = 0.0f; c <= 1.0f; c += 0.1f) {
        EXPECT_NEAR(c, bankCollectiveFromThrust(bankThrustFromCollective(c, 5.7f), 5.7f), 1e-5f);
    }
    EXPECT_FLOAT_EQ(1.0f, bankCollectiveFromThrust(10.0f, 5.7f));
}

TEST(BankModeTest, RotatingByTheNoseOffsetAndBackIsTheIdentity)
{
    float forward = 1.0f;
    float right = 0.0f;
    // the nose 90 degrees left of the track sees the track to its right
    bankRotate(&forward, &right, 90.0f * DEG);
    EXPECT_NEAR(0.0f, forward, 1e-6f);
    EXPECT_NEAR(1.0f, right, 1e-6f);

    forward = 0.3f;
    right = -0.7f;
    bankRotate(&forward, &right, 0.6f);
    bankRotate(&forward, &right, -0.6f);
    EXPECT_NEAR(0.3f, forward, 1e-6f);
    EXPECT_NEAR(-0.7f, right, 1e-6f);
}

TEST(BankModeTest, TheTiltKeepsAQuarterOfTheSpareThrust)
{
    EXPECT_FLOAT_EQ(1.0f + 0.75f * 4.7f, bankTiltThrust(1.0f, 5.7f));
    EXPECT_FLOAT_EQ(2.0f, bankTiltThrust(2.0f, 1.5f));
}

TEST(BankModeTest, BankTakesTheHorizontalForceBeforeSpeed)
{
    const float horizontal = 3.0f;
    float forward;
    float right;

    // no bank: the forward force gets it all
    bankThrustVector(1.0f, horizontal, 0.0f, 10.0f, &forward, &right);
    EXPECT_FLOAT_EQ(0.0f, right);
    EXPECT_FLOAT_EQ(horizontal, forward);

    // half the stick is a 45 degree bank, and the forward force gets what is left
    bankThrustVector(1.0f, horizontal, 0.5f, 10.0f, &forward, &right);
    EXPECT_NEAR(1.0f, right, 1e-5f);
    EXPECT_NEAR(sqrtf(sq(horizontal) - 1.0f), forward, 1e-5f);

    // full Bank takes the whole budget, left is negative
    bankThrustVector(1.0f, horizontal, -1.0f, 10.0f, &forward, &right);
    EXPECT_NEAR(-horizontal, right, 1e-4f);
    EXPECT_NEAR(0.0f, forward, 1e-2f);

    // braking is limited to a 30 degree tilt back, and to what the bank leaves
    bankThrustVector(2.0f, horizontal, 0.0f, -10.0f, &forward, &right);
    EXPECT_NEAR(-2.0f * tanf(30.0f * DEG), forward, 1e-5f);
    bankThrustVector(1.0f, horizontal, 1.0f, -10.0f, &forward, &right);
    EXPECT_NEAR(0.0f, forward, 1e-2f);
}

TEST(BankModeTest, SpeedStickIsASpeedInTheLowerHalfAndAForceInTheUpperHalf)
{
    const float mid = 0.5f;
    const float maxForce = 3.0f;
    // the bottom brakes in proportion to the speed
    EXPECT_FLOAT_EQ(-2.0f * 0.3f, bankSpeedForce(0.0f, 0.3f, mid, maxForce));
    // at the target speed the force is the feedforward that holds it
    EXPECT_FLOAT_EQ(0.25f, bankSpeedForce(0.25f, 0.25f, mid, maxForce));
    // above a third of the stick a craft that is too fast coasts: no braking force
    EXPECT_FLOAT_EQ(0.4f, bankSpeedForce(0.4f, 0.8f, mid, maxForce));
    // below a third it is braked
    EXPECT_LT(bankSpeedForce(0.3f, 0.8f, mid, maxForce), 0.0f);
    // continuous at the centre, and the top is all the force there is
    EXPECT_NEAR(bankSpeedForce(0.5f, 0.2f, mid, maxForce), bankSpeedForce(0.5001f, 0.2f, mid, maxForce), 1e-3f);
    EXPECT_FLOAT_EQ(maxForce, bankSpeedForce(1.0f, 0.2f, mid, maxForce));
}

TEST(BankModeTest, DragIsLevelledWithTheAttitude)
{
    float forward;
    float left;
    // pitched forward 40 and banked 30 in level flight, a pure forward drag of 0.2 g reads on both body axes
    const float pitch = 40.0f * DEG;
    const float roll = 30.0f * DEG;
    const float accX = -0.2f * cosf(pitch);
    const float accY = -0.2f * sinf(pitch) * sinf(roll);
    bankLevelDrag(accX, accY, pitch, roll, &forward, &left);
    EXPECT_NEAR(-0.2f, forward, 1e-5f);
    EXPECT_NEAR(0.0f, left, 1e-5f);
}

TEST(BankModeTest, TravelDirectionFlipsOnlyForClearRearwardDrag)
{
    EXPECT_FLOAT_EQ(1.0f, bankTravelDirection(-0.3f));
    EXPECT_FLOAT_EQ(1.0f, bankTravelDirection(0.0f));
    EXPECT_FLOAT_EQ(-1.0f, bankTravelDirection(0.3f));
    EXPECT_FLOAT_EQ(0.0f, bankTravelDirection(0.03f));
}

TEST(BankModeTest, TurnFeedforwardIsTheCoordinatedRateWithinItsShare)
{
    // a right bank turns clockwise (negative), faster the slower the craft
    EXPECT_LT(bankTurnFeedforward(1.0f, 0.5f), 0.0f);
    EXPECT_NEAR(2.0f * bankTurnFeedforward(0.5f, 0.5f), bankTurnFeedforward(0.5f, 0.25f), 1e-3f);
    // reversing turns the other way, like a car
    EXPECT_GT(bankTurnFeedforward(1.0f, -0.5f), 0.0f);
    // no speed, no feedforward, and never more than 60 % of the yaw limit
    EXPECT_FLOAT_EQ(0.0f, bankTurnFeedforward(1.0f, 0.0f));
    EXPECT_FLOAT_EQ(-0.6f * 360.0f, bankTurnFeedforward(11.0f, 0.1f));
}

TEST(BankModeTest, ThrustTrimTakesOffExcessAndAddsMissingThrustOnlyBelowOneG)
{
    float trim = 1.0f;
    for (int i = 0; i < 300; i++) {
        trim = bankUpdateThrustTrim(trim, 1.5f, 1.0f, 0.01f);
    }
    EXPECT_NEAR(1.5f, trim, 1e-3f);

    // sinking: the props make less than modelled, the trim adds it
    trim = 1.0f;
    for (int i = 0; i < 300; i++) {
        trim = bankUpdateThrustTrim(trim, 0.8f, 1.0f, 0.01f);
    }
    EXPECT_NEAR(0.8f, trim, 1e-3f);

    // at 1 g it may be on the ground: it only moves back towards 1
    for (int i = 0; i < 300; i++) {
        trim = bankUpdateThrustTrim(trim, 1.0f, 2.0f, 0.01f);
    }
    EXPECT_NEAR(0.8f, trim, 1e-3f);
}

TEST(BankModeTest, OnlyATrimThatTakesThrustOffLeaksBack)
{
    float trim = 1.5f;
    float adding = 0.5f;
    for (int i = 0; i < 300; i++) {
        trim = bankLeakThrustTrim(trim, 0.01f);
        adding = bankLeakThrustTrim(adding, 0.01f);
    }
    EXPECT_NEAR(1.0f, trim, 0.03f);
    EXPECT_FLOAT_EQ(0.5f, adding);
}

TEST(BankModeTest, ThrustTrimAddsLessAtOnceWhenTheInflowFalls)
{
    EXPECT_FLOAT_EQ(0.5f, bankScaleTrimWithInflow(0.5f, 0.6f, 0.4f));
    EXPECT_FLOAT_EQ(1.2f, bankScaleTrimWithInflow(1.2f, 0.0f, 0.4f));
    EXPECT_NEAR(1.0f - 0.5f * 0.1f / 0.5f, bankScaleTrimWithInflow(0.5f, 0.0f, 0.4f), 1e-6f);
}

TEST(BankModeTest, LookIntoTheTurnIsMoreTheMoreBankAgainstSpeed)
{
    EXPECT_FLOAT_EQ(0.0f, bankLookIntoTurnOffset(0.0f, 0.5f));
    EXPECT_NEAR(-45.0f * DEG, bankLookIntoTurnOffset(1.0f, 1.0f), 1e-5f);
    EXPECT_NEAR(90.0f * DEG, bankLookIntoTurnOffset(-1.0f, 0.0f), 1e-5f);
    EXPECT_NEAR(0.5f * atan2f(0.1f, 1.0f), fabsf(bankLookIntoTurnOffset(0.1f, 1.0f)), 1e-5f);
}

TEST(BankModeTest, CentredSticksHoverLevel)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    computeN(&in, &state, &out, 500);
    EXPECT_NEAR(hoverCollective(), out.collective, 2e-3f);
    EXPECT_NEAR(0.0f, out.pitchDeg, 1e-3f);
    EXPECT_NEAR(0.0f, out.rollDeg, 1e-3f);
    EXPECT_NEAR(0.0f, out.yawRateDps, 1e-3f);
}

TEST(BankModeTest, LiftIsKeptAtTheMeasuredTilt)
{
    pgResetAll();
    for (float tiltDeg = 0.0f; tiltDeg <= 60.0f; tiltDeg += 20.0f) {
        bankState_t state = {};
        bankOutputs_t out = {};
        bankInputs_t in = centredInputs();
        in.armed = false;       // no trim
        in.measuredPitchDeg = tiltDeg;
        in.measuredRollDeg = 10.0f;
        computeN(&in, &state, &out, 10);
        const float tiltCos = cosf(tiltDeg * DEG) * cosf(10.0f * DEG);
        EXPECT_NEAR(1.0f, bankThrustFromCollective(out.collective, thrustToWeight()) * tiltCos, 1e-3f);
    }
}

TEST(BankModeTest, CollectiveStopsAtTheTiltBudgetAndIsZeroInverted)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = false;
    in.measuredPitchDeg = 84.0f;
    computeN(&in, &state, &out, 10);
    EXPECT_NEAR(bankTiltThrust(1.0f, thrustToWeight()), bankThrustFromCollective(out.collective, thrustToWeight()), 1e-3f);

    in.measuredRollDeg = 120.0f;
    computeN(&in, &state, &out, 10);
    EXPECT_FLOAT_EQ(0.0f, out.collective);
}

TEST(BankModeTest, BankRightRollsRightAndSpeedPitchesForward)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.roll = 0.5f;
    in.speed = 0.75f;
    computeN(&in, &state, &out, 10);
    EXPECT_GT(out.rollDeg, 30.0f);
    EXPECT_GT(out.pitchDeg, 10.0f);
}

TEST(BankModeTest, TheHeadCheckTurnsTheNoseButNotTheThrustInTheWorld)
{
    pgResetAll();
    bankState_t plain = {};
    bankState_t looked = {};
    bankOutputs_t plainOut = {};
    bankOutputs_t lookedOut = {};
    bankInputs_t in = centredInputs();
    in.armed = false;
    in.roll = 0.3f;
    in.speed = 0.75f;
    computeN(&in, &plain, &plainOut, 100);
    in.yaw = 1.0f;
    computeN(&in, &looked, &lookedOut, 100);
    EXPECT_NEAR(90.0f * DEG, looked.noseOffset, 1e-4f);

    float forward;
    float right;
    targetThrust(&lookedOut, &forward, &right);
    bankRotate(&forward, &right, -looked.noseOffset);
    float plainForward;
    float plainRight;
    targetThrust(&plainOut, &plainForward, &plainRight);
    EXPECT_NEAR(plainForward, forward, 1e-3f);
    EXPECT_NEAR(plainRight, right, 1e-3f);
    EXPECT_NEAR(plainOut.collective, lookedOut.collective, 1e-5f);
}

TEST(BankModeTest, TheHeadCheckSlewsAtTheYawLimitAndYawsWithIt)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.yaw = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_NEAR(3.6f * DEG, state.noseOffset, 1e-5f);
    EXPECT_NEAR(360.0f, out.yawRateDps, 1e-2f);
}

TEST(BankModeTest, LiftAtTheBottomTiltsNoFurtherThanHalfAG)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t bottom = {};
    bankOutputs_t quarter = {};
    bankInputs_t in = centredInputs();
    in.armed = false;
    in.speed = 0.5f;
    in.roll = 0.3f;
    in.lift = 0.0f;
    computeN(&in, &state, &bottom, 10);
    EXPECT_FLOAT_EQ(0.0f, bottom.collective);
    state = {};
    in.lift = 0.5f - 0.5f * (0.05f + 0.5f * 0.95f);    // half a g after the deadband
    computeN(&in, &state, &quarter, 10);
    EXPECT_NEAR(quarter.pitchDeg, bottom.pitchDeg, 0.5f);
    EXPECT_NEAR(quarter.rollDeg, bottom.rollDeg, 0.5f);
}

TEST(BankModeTest, ForwardDragReadsAsSpeedAndTheBottomOfTheStickBrakes)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = false;
    in.accX = -0.3f;        // drag of flying forwards
    computeN(&in, &state, &out, 300);
    EXPECT_NEAR(-30.0f, out.pitchDeg, 1.0f);
}

TEST(BankModeTest, ABankedTurnFeedsTheYawForwardFromTheBankTheCraftIsIn)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = false;
    in.speed = 0.5f;
    in.roll = 0.5f;
    in.accX = -0.3f;
    computeN(&in, &state, &out, 300);
    const float levelYaw = out.yawRateDps;
    in.measuredRollDeg = 45.0f;
    computeN(&in, &state, &out, 1);
    EXPECT_LT(out.yawRateDps, levelYaw - 30.0f);
}

TEST(BankModeTest, NothingTurnsWithLiftAtTheBottom)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.lift = 0.0f;
    in.accY = -0.3f;
    in.measuredRollDeg = 30.0f;
    computeN(&in, &state, &out, 100);
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
    memset(&attitude, 0, sizeof(attitude));
    acc.dev.acc_1G_rec = 1.0f / 512.0f;
    rcCommand[THROTTLE] = 1000;     // Speed stick at the bottom
    testThrottleStatus = THROTTLE_LOW;
    DISABLE_ARMING_FLAG(ARMED);
    testMotorMixRange = 0.0f;
}

TEST(BankModeTest, EngagedWithTheThrottleLowNoTiltUntilTheBankStickIsCentred)
{
    setUpUpdate();
    ENABLE_FLIGHT_MODE(BANK_MODE);
    testDeflection[FD_ROLL] = 1.0f;
    timeUs_t now = 1000;
    for (int i = 0; i < 10; i++, now += 10000) {
        bankModeUpdate(now);
    }
    EXPECT_FLOAT_EQ(0.0f, bankModeGetAngleTarget(FD_ROLL));

    testDeflection[FD_ROLL] = 0.0f;
    bankModeUpdate(now);
    now += 10000;
    testDeflection[FD_ROLL] = 1.0f;
    bankModeUpdate(now);
    EXPECT_GT(bankModeGetAngleTarget(FD_ROLL), 0.0f);
}

TEST(BankModeTest, ThrottleIsLowOnlyWithNoCollective)
{
    setUpUpdate();
    ENABLE_FLIGHT_MODE(BANK_MODE);
    bankModeUpdate(1000);
    EXPECT_FALSE(bankModeIsThrottleLow());
    testDeflection[FD_PITCH] = -1.0f;     // Lift at the bottom
    bankModeUpdate(11000);
    EXPECT_TRUE(bankModeIsThrottleLow());
}

TEST(BankModeTest, UpdateMirrorsTheModeFlagAndClearsWhenInactive)
{
    setUpUpdate();
    bankModeUpdate(1000);
    EXPECT_FALSE(bankModeIsActive());

    ENABLE_FLIGHT_MODE(BANK_MODE);
    bankModeUpdate(2000);
    EXPECT_TRUE(bankModeIsActive());
    EXPECT_NEAR(hoverCollective(), bankModeGetCollective(), 1e-5f);

    testDeflection[FD_ROLL] = 1.0f;
    bankModeUpdate(12000);
    EXPECT_GT(bankModeGetAngleTarget(FD_ROLL), 0.0f);

    DISABLE_FLIGHT_MODE(BANK_MODE);
    bankModeUpdate(22000);
    EXPECT_FALSE(bankModeIsActive());
    EXPECT_FLOAT_EQ(0.0f, bankModeGetAngleTarget(FD_ROLL));
    EXPECT_FLOAT_EQ(0.0f, bankModeGetYawRate());
    EXPECT_FLOAT_EQ(0.0f, bankModeGetCollective());
}

TEST(BankModeTest, UpdateReadsTheDragFromTheAccelerometer)
{
    setUpUpdate();
    ENABLE_FLIGHT_MODE(BANK_MODE);
    // drag towards the right (-Y) of 0.3 g: the craft is drifting left, so the nose turns left
    acc.accADC.y = -0.3f * 512.0f;
    acc.accADC.z = 512.0f;
    timeUs_t now = 1000;
    for (int i = 0; i < 100; i++, now += 10000) {
        bankModeUpdate(now);
    }
    EXPECT_GT(bankModeGetYawRate(), 0.0f);
}

TEST(BankModeTest, DebugModeBankLogsTheEstimates)
{
    setUpUpdate();
    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_FLIGHT_MODE(BANK_MODE);
    debugMode = DEBUG_BANK;
    memset(debug, 0, sizeof(debug));
    acc.accADC.x = -0.3f * 512.0f;      // flying forwards
    rcCommand[THROTTLE] = 1500;         // half the Speed stick
    testThrottleStatus = THROTTLE_HIGH;
    timeUs_t now = 1000;
    for (int i = 0; i < 200; i++, now += 10000) {
        acc.accADC.z = bankThrustFromCollective(bankModeGetCollective(), thrustToWeight()) * 512.0f;
        bankModeUpdate(now);
    }
    EXPECT_GT(debug[0], 200);           // speed estimate, 0.001 g
    EXPECT_GT(debug[1], 0);             // target speed
    EXPECT_NEAR(thrustToWeight() * 100.0f, debug[4], 1.0f);
    debugMode = DEBUG_NONE;
}
