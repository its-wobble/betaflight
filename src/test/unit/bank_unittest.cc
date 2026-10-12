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
#include <stdint.h>

extern "C" {
    #include "platform.h"

    #include "common/axis.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"
    #include "flight/bank_mode.h"
    #include "io/beeper.h"
    #include "pg/bank_mode.h"
    #include "pg/pg.h"
    #include "sensors/acceleration.h"

    extern uint8_t __config_start;
    extern uint8_t __config_end;

    acc_t acc;
    float rcCommand[4];

    float testDeflection[3];
    throttleStatus_e testThrottleStatus = THROTTLE_HIGH;

    void beeperConfirmationBeeps(uint8_t) {}
    float getRcDeflection(int axis) { return testDeflection[axis]; }
    throttleStatus_e calculateThrottleStatus(void) { return testThrottleStatus; }

    float bankShapeStick(float deflection, float deadband);
    float bankLimitTilt(float fz, float *theta, float *phi, float thrustToWeight);
    float bankLiftFromStick(float stick, float thrustToWeight);
    float bankCalcCollective(float fz, float theta, float phi, float thrustToWeight);
    float bankSlewHeadCheck(float offset, float target, float rate, float dt);
    void bankRotateTilt(float *theta, float *phi, float offset);
    float bankCalcAlignYawRate(float dragX, float dragY, float offset, float gainPerSecond, float dragThreshold);
    void bankCompute(const bankModeConfig_t *config, const bankInputs_t *in, bankState_t *state, bankOutputs_t *out);
}

#include "gtest/gtest.h"

static const float DEG = M_PI / 180.0f;

TEST(BankModeTest, ConfigDefaults)
{
    pgResetAll();
    EXPECT_EQ(200, bankModeConfig()->thrustToWeight);
    EXPECT_EQ(45, bankModeConfig()->maxBankAngle);
    EXPECT_EQ(30, bankModeConfig()->maxTiltAngle);
}

TEST(BankModeTest, ModeBitIsDistinct)
{
    const uint16_t others = ANGLE_MODE | HORIZON_MODE | MAG_MODE | ALT_HOLD_MODE | POS_HOLD_MODE
        | HEADFREE_MODE | CHIRP_MODE | PASSTHRU_MODE | FAILSAFE_MODE | GPS_RESCUE_MODE
        | AUTOPILOT_MODE | LAUNCH_MODE;
    EXPECT_EQ(0, BANK_MODE & others);
    EXPECT_EQ(BANK_MODE, 1 << 14);
}

TEST(BankModeTest, BoxIsMappedLikeAnAttitudeMode)
{
    const uint8_t boxIdToFlightModeMap[] = BOXID_TO_FLIGHT_MODE_MAP_INITIALIZER;

    EXPECT_LE(BOXBANK, BOXID_FLIGHTMODE_LAST);
    EXPECT_EQ(BANK_MODE, 1 << boxIdToFlightModeMap[BOXBANK]);
}

TEST(BankModeTest, StickShapingRemovesDeadbandAndKeepsFullScale)
{
    EXPECT_FLOAT_EQ(0.0f, bankShapeStick(0.04f, 0.05f));
    EXPECT_FLOAT_EQ(0.0f, bankShapeStick(-0.05f, 0.05f));
    EXPECT_FLOAT_EQ(1.0f, bankShapeStick(1.0f, 0.05f));
    EXPECT_FLOAT_EQ(-1.0f, bankShapeStick(-1.5f, 0.05f));
    EXPECT_NEAR(0.5f, bankShapeStick(0.525f, 0.05f), 1e-5f);
    EXPECT_NEAR(-0.5f, bankShapeStick(-0.525f, 0.05f), 1e-5f);
}

TEST(BankModeTest, CollectiveIsHoverAtCentreAndCompensatesTilt)
{
    const float tw = 2.0f;
    EXPECT_NEAR(0.5f, bankCalcCollective(0.0f, 0.0f, 0.0f, tw), 1e-5f);
    // forward pitch with no vertical command raises thrust by 1 / cos(theta)
    const float theta = 30.0f * DEG;
    EXPECT_NEAR(0.5f / cosf(theta), bankCalcCollective(0.0f, theta, 0.0f, tw), 1e-5f);
    // bank and pitch together
    const float phi = 20.0f * DEG;
    EXPECT_NEAR(0.5f / (cosf(theta) * cosf(phi)), bankCalcCollective(0.0f, theta, phi, tw), 1e-5f);
    // the vertical component of the thrust stays the weight: T * cos(tilt) == 1 + Fz
    const float t = bankCalcCollective(0.2f, theta, phi, tw) * tw;
    EXPECT_NEAR(1.2f, t * cosf(theta) * cosf(phi), 1e-5f);
}

TEST(BankModeTest, TiltLimiterKeepsFzAndStaysWithinMaxThrust)
{
    const float tw = 1.5f;
    float theta = 40.0f * DEG;
    float phi = 40.0f * DEG;
    const float fz = bankLimitTilt(0.1f, &theta, &phi, tw);
    EXPECT_FLOAT_EQ(0.1f, fz);
    EXPECT_LE(bankCalcCollective(fz, theta, phi, tw), 1.0f + 1e-5f);
    // the collective is saturated exactly, none of the budget is left unused
    EXPECT_NEAR(1.0f, bankCalcCollective(fz, theta, phi, tw), 1e-4f);
}

TEST(BankModeTest, TiltLimiterGivesBankPriorityOverForwardTilt)
{
    const float tw = 1.5f;
    float theta = 40.0f * DEG;
    float phi = 30.0f * DEG;
    bankLimitTilt(0.0f, &theta, &phi, tw);
    // bank alone fits the budget (cos 30 deg = 0.87 > 1 / 1.5), so it is untouched
    EXPECT_NEAR(30.0f * DEG, phi, 1e-5f);
    // forward tilt takes the rest, so it is trimmed
    EXPECT_LT(theta, 40.0f * DEG);
    EXPECT_GT(theta, 0.0f);
}

TEST(BankModeTest, TiltLimiterTrimsBankOnlyWhenItAloneExceedsBudget)
{
    const float tw = 1.2f;
    float theta = 30.0f * DEG;
    float phi = 45.0f * DEG;
    bankLimitTilt(0.0f, &theta, &phi, tw);
    // budget cos >= 1 / 1.2 gives a maximum bank of acos(0.8333) = 33.56 deg, and nothing is left for pitch
    EXPECT_NEAR(acosf(1.0f / 1.2f), phi, 1e-4f);
    EXPECT_NEAR(0.0f, theta, 1e-3f);
}

TEST(BankModeTest, TiltLimiterKeepsSigns)
{
    float theta = -40.0f * DEG;
    float phi = -30.0f * DEG;
    bankLimitTilt(0.0f, &theta, &phi, 1.5f);
    EXPECT_LT(theta, 0.0f);
    EXPECT_LT(phi, 0.0f);
}

TEST(BankModeTest, TiltLimiterCapsFzWhenThrustCannotLiftIt)
{
    float theta = 20.0f * DEG;
    float phi = 20.0f * DEG;
    const float fz = bankLimitTilt(2.0f, &theta, &phi, 1.5f);
    EXPECT_FLOAT_EQ(0.5f, fz);
    EXPECT_NEAR(0.0f, theta, 1e-3f);
    EXPECT_NEAR(0.0f, phi, 1e-3f);
}

TEST(BankModeTest, TiltLimiterAllowsFullDescent)
{
    float theta = 20.0f * DEG;
    float phi = 20.0f * DEG;
    const float fz = bankLimitTilt(-2.0f, &theta, &phi, 2.0f);
    EXPECT_FLOAT_EQ(-1.0f, fz);
    EXPECT_FLOAT_EQ(0.0f, bankCalcCollective(fz, theta, phi, 2.0f));
}

TEST(BankModeTest, HeadCheckSlewsAndReturns)
{
    float offset = 0.0f;
    const float rate = 100.0f * DEG;
    offset = bankSlewHeadCheck(offset, 90.0f * DEG, rate, 0.1f);
    EXPECT_NEAR(10.0f * DEG, offset, 1e-5f);
    for (int i = 0; i < 20; i++) {
        offset = bankSlewHeadCheck(offset, 90.0f * DEG, rate, 0.1f);
    }
    EXPECT_NEAR(90.0f * DEG, offset, 1e-5f);
    for (int i = 0; i < 20; i++) {
        offset = bankSlewHeadCheck(offset, 0.0f, rate, 0.1f);
    }
    EXPECT_NEAR(0.0f, offset, 1e-5f);
}

TEST(BankModeTest, RotatingTheTiltBackRecoversIt)
{
    for (float offsetDeg = -170.0f; offsetDeg <= 170.0f; offsetDeg += 20.0f) {
        float theta = 12.0f * DEG;
        float phi = -25.0f * DEG;
        bankRotateTilt(&theta, &phi, offsetDeg * DEG);
        bankRotateTilt(&theta, &phi, -offsetDeg * DEG);
        EXPECT_NEAR(12.0f * DEG, theta, 1e-4f);
        EXPECT_NEAR(-25.0f * DEG, phi, 1e-4f);
    }
}

TEST(BankModeTest, RotatingTheTiltKeepsItsSizeAndTheCollective)
{
    const float tw = 2.0f;
    const float theta0 = 15.0f * DEG;
    const float phi0 = 25.0f * DEG;
    const float before = bankCalcCollective(0.1f, theta0, phi0, tw);
    for (float offsetDeg = -120.0f; offsetDeg <= 120.0f; offsetDeg += 30.0f) {
        float theta = theta0;
        float phi = phi0;
        bankRotateTilt(&theta, &phi, offsetDeg * DEG);
        EXPECT_NEAR(cosf(theta0) * cosf(phi0), cosf(theta) * cosf(phi), 1e-5f);
        EXPECT_NEAR(before, bankCalcCollective(0.1f, theta, phi, tw), 1e-5f);
    }
}

TEST(BankModeTest, ZeroOffsetLeavesTheTiltAlone)
{
    float theta = 12.0f * DEG;
    float phi = -25.0f * DEG;
    bankRotateTilt(&theta, &phi, 0.0f);
    EXPECT_NEAR(12.0f * DEG, theta, 1e-5f);
    EXPECT_NEAR(-25.0f * DEG, phi, 1e-5f);
}

TEST(BankModeTest, QuarterTurnOffsetSwapsBankAndPitchAxes)
{
    // a pure bank in the travel frame, with the nose 90 degrees counter-clockwise from the travel
    // direction, is a pure pitch in the body frame
    float theta = 0.0f;
    float phi = 20.0f * DEG;
    bankRotateTilt(&theta, &phi, 90.0f * DEG);
    EXPECT_NEAR(0.0f, phi, 1e-4f);
    EXPECT_NEAR(20.0f * DEG, fabsf(theta), 1e-4f);
}

TEST(BankModeTest, YawAlignmentIgnoresLowDrag)
{
    EXPECT_FLOAT_EQ(0.0f, bankCalcAlignYawRate(0.02f, 0.02f, 0.0f, 4.0f, 0.05f));
    EXPECT_FLOAT_EQ(0.0f, bankCalcAlignYawRate(0.0f, 0.0f, 0.0f, 4.0f, 0.0f));
}

TEST(BankModeTest, YawAlignmentTurnsTheNoseTowardsTheTravelDirection)
{
    // drifting to the left: the drag points to the right (-Y), so the nose should turn left (positive)
    EXPECT_GT(bankCalcAlignYawRate(0.0f, -0.2f, 0.0f, 4.0f, 0.05f), 0.0f);
    // drifting to the right: nose turns right (negative)
    EXPECT_LT(bankCalcAlignYawRate(0.0f, 0.2f, 0.0f, 4.0f, 0.05f), 0.0f);
    // moving straight forward: drag points back (-X), nothing to do
    EXPECT_NEAR(0.0f, bankCalcAlignYawRate(-0.2f, 0.0f, 0.0f, 4.0f, 0.05f), 1e-3f);
    // proportional: 90 degrees at 4 /s is 4 * pi / 2 rad/s
    EXPECT_NEAR(4.0f * 90.0f, bankCalcAlignYawRate(0.0f, -0.5f, 0.0f, 4.0f, 0.05f), 1e-2f);
}

TEST(BankModeTest, YawAlignmentFadesInWithDrag)
{
    const float low = fabsf(bankCalcAlignYawRate(0.0f, -0.06f, 0.0f, 4.0f, 0.05f));
    const float high = fabsf(bankCalcAlignYawRate(0.0f, -0.2f, 0.0f, 4.0f, 0.05f));
    EXPECT_GT(low, 0.0f);
    EXPECT_LT(low, high);
}

TEST(BankModeTest, YawAlignmentAlignsTheTailWhenReversing)
{
    // moving backwards: the drag points forward (+X); the tail should follow, so no half-turn
    EXPECT_NEAR(0.0f, bankCalcAlignYawRate(0.2f, 0.0f, 0.0f, 4.0f, 0.05f), 1e-3f);
    // reversing and drifting to the left: the tail moves left, so the nose turns right (negative)
    EXPECT_LT(bankCalcAlignYawRate(0.2f, -0.1f, 0.0f, 4.0f, 0.05f), 0.0f);
}

TEST(BankModeTest, YawAlignmentTargetsTheHeadCheckOffset)
{
    // travelling straight ahead with the nose already at the offset: nothing to correct
    // (travel is 30 degrees to the right of the nose)
    const float beta = -30.0f * DEG;
    const float dragX = -0.2f * cosf(beta);
    const float dragY = -0.2f * sinf(beta);
    EXPECT_NEAR(0.0f, bankCalcAlignYawRate(dragX, dragY, 30.0f * DEG, 4.0f, 0.05f), 1e-2f);
    EXPECT_LT(bankCalcAlignYawRate(dragX, dragY, 0.0f, 4.0f, 0.05f), 0.0f);
}

static bankInputs_t centredInputs(void)
{
    // right stick centred, throttle at the centre (1 g of lift, hover)
    bankInputs_t in = {};
    in.vertical = 0.5f;
    in.dt = 0.01f;
    return in;
}

TEST(BankModeTest, ComputeAtCentreHoversWithNoTilt)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    const bankInputs_t in = centredInputs();
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_NEAR(0.0f, out.rollDeg, 1e-5f);
    EXPECT_NEAR(0.0f, out.pitchDeg, 1e-5f);
    EXPECT_NEAR(0.0f, out.yawRateDps, 1e-5f);
    EXPECT_NEAR(0.5f, out.collective, 1e-5f);
}

TEST(BankModeTest, ComputeSignsFollowTheSticks)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.roll = 1.0f;
    in.pitch = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_GT(out.rollDeg, 0.0f);    // roll right
    EXPECT_GT(out.pitchDeg, 0.0f);   // nose-down, forward
    // forward pitch adds collective so the vertical force is unchanged
    EXPECT_GT(out.collective, 0.5f);
}

TEST(BankModeTest, LiftIsNoneAtTheBottomOneGAtTheCentreAndMaxThrustAtTheTop)
{
    for (float tw = 1.1f; tw <= 6.0f; tw += 0.7f) {
        EXPECT_FLOAT_EQ(0.0f, bankLiftFromStick(0.0f, tw));
        EXPECT_NEAR(1.0f, bankLiftFromStick(0.5f, tw), 1e-6f);
        EXPECT_NEAR(tw, bankLiftFromStick(1.0f, tw), 1e-5f);
        // linear in each half
        EXPECT_NEAR(0.5f, bankLiftFromStick(0.25f, tw), 1e-6f);
        EXPECT_NEAR(1.0f + 0.5f * (tw - 1.0f), bankLiftFromStick(0.75f, tw), 1e-5f);
    }
    // out-of-range sticks are clamped
    EXPECT_FLOAT_EQ(0.0f, bankLiftFromStick(-0.3f, 2.0f));
    EXPECT_NEAR(2.0f, bankLiftFromStick(1.3f, 2.0f), 1e-6f);
}

TEST(BankModeTest, ComputeThrottleTopIsAllMaxThrustAsLift)
{
    pgResetAll();
    for (int tw = 110; tw <= 600; tw += 70) {
        bankModeConfig_t config = *bankModeConfig();
        config.thrustToWeight = tw;
        bankState_t state = {};
        bankOutputs_t out = {};
        bankInputs_t in = centredInputs();
        in.vertical = 1.0f;
        in.roll = 1.0f;
        in.pitch = 1.0f;
        bankCompute(&config, &in, &state, &out);
        // all the thrust is lift, so there is no budget left for tilt
        EXPECT_NEAR(1.0f, out.collective, 1e-4f);
        EXPECT_NEAR(0.0f, out.rollDeg, 1e-2f);
        EXPECT_NEAR(0.0f, out.pitchDeg, 1e-2f);
    }
}

TEST(BankModeTest, ComputeThrottleBottomIsFreeFallAndCentreIsHover)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();

    in.vertical = 0.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(0.0f, out.collective);          // no lift, the craft drops with gravity

    in.vertical = 0.5f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_NEAR(0.5f, out.collective, 1e-5f);       // thrust equals weight, 1 / (T/W)

    in.vertical = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_NEAR(1.0f, out.collective, 1e-5f);       // 2 g of lift at T/W = 2
}

TEST(BankModeTest, ComputeThrottleIsLinearInLift)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t a = {};
    bankOutputs_t b = {};
    bankInputs_t in = centredInputs();
    in.vertical = 0.25f;
    bankCompute(bankModeConfig(), &in, &state, &a);
    in.vertical = 0.5f;
    bankCompute(bankModeConfig(), &in, &state, &b);
    EXPECT_NEAR(b.collective, 2.0f * a.collective, 1e-5f);
}

TEST(BankModeTest, ComputeBottomStickWithTiltStillGivesNoLift)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.vertical = 0.0f;
    in.roll = 1.0f;
    in.pitch = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(0.0f, out.collective);
}

TEST(BankModeTest, ComputeRightStickCentreMeansNoTiltOrYaw)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.roll = 0.04f;    // inside the deadband
    in.pitch = -0.04f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(0.0f, out.rollDeg);
    EXPECT_FLOAT_EQ(0.0f, out.pitchDeg);
}

TEST(BankModeTest, EngagingOnTheGroundWithTheRightStickOffCentreHoldsTiltUntilCentred)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.throttleLow = true;
    in.vertical = 0.0f;
    in.roll = 1.0f;
    in.pitch = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(0.0f, out.rollDeg);
    EXPECT_FLOAT_EQ(0.0f, out.pitchDeg);

    // still held while the stick stays deflected, even once the throttle is raised
    in.throttleLow = false;
    in.vertical = 0.5f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(0.0f, out.rollDeg);

    // centring releases it
    in.roll = 0.0f;
    in.pitch = 0.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    in.roll = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_GT(out.rollDeg, 0.0f);
}

TEST(BankModeTest, EngagingInFlightIsNotHeld)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.roll = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_GT(out.rollDeg, 0.0f);
}

TEST(BankModeTest, ComputeThrottleLowCutsTheCollective)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.throttleLow = true;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(0.0f, out.collective);
}

TEST(BankModeTest, ComputeFullSticksNeverExceedMaximumThrust)
{
    pgResetAll();
    for (int tw = 110; tw <= 400; tw += 10) {
        bankModeConfig_t config = *bankModeConfig();
        config.thrustToWeight = tw;
        for (float vertical = 0.0f; vertical <= 1.0f; vertical += 0.25f) {
            bankState_t state = {};
            bankOutputs_t out = {};
            bankInputs_t in = centredInputs();
            in.roll = 1.0f;
            in.pitch = -1.0f;
            in.vertical = vertical;
            bankCompute(&config, &in, &state, &out);
            EXPECT_LE(out.collective, 1.0f);
            EXPECT_GE(out.collective, 0.0f);
        }
    }
}

TEST(BankModeTest, ComputeHeadCheckLeavesCollectiveAndYawsInHover)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.roll = 0.5f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    const float collectiveBefore = out.collective;
    in.yaw = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_NEAR(collectiveBefore, out.collective, 1e-5f);
    // the slew rate drives the nose in the stick direction while the drag is too low to use
    EXPECT_GT(out.yawRateDps, 0.0f);
    EXPECT_GT(state.headCheckOffset, 0.0f);
}

TEST(BankModeTest, ComputeYawRateIsLimited)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.dragY = -1.0f;
    in.yaw = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_LE(out.yawRateDps, bankModeConfig()->yawMaxRate);
    EXPECT_GE(out.yawRateDps, -(float)bankModeConfig()->yawMaxRate);
}

static void setUpUpdate(void)
{
    pgResetAll();
    flightModeFlags = 0;
    bankModeUpdate(0);
    memset(testDeflection, 0, sizeof(testDeflection));
    memset(&acc, 0, sizeof(acc));
    acc.dev.acc_1G_rec = 1.0f / 512.0f;
    rcCommand[THROTTLE] = 1500;
    testThrottleStatus = THROTTLE_HIGH;
}

TEST(BankModeTest, UpdateMirrorsTheModeFlagAndClearsWhenInactive)
{
    setUpUpdate();
    bankModeUpdate(1000);
    EXPECT_FALSE(bankModeIsActive());

    ENABLE_FLIGHT_MODE(BANK_MODE);
    bankModeUpdate(2000);
    EXPECT_TRUE(bankModeIsActive());
    EXPECT_NEAR(0.5f, bankModeGetCollective(), 1e-5f);

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
    timeUs_t now = 1000;
    for (int i = 0; i < 100; i++) {
        bankModeUpdate(now);
        now += 10000;
    }
    EXPECT_GT(bankModeGetYawRate(), 0.0f);
    DISABLE_FLIGHT_MODE(BANK_MODE);
}
