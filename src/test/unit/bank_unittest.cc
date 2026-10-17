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
    #include "flight/imu.h"
    #include "io/beeper.h"
    #include "pg/bank_mode.h"
    #include "build/debug.h"
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

    float bankShapeStick(float deflection, float deadband);
    float bankLimitTilt(float fz, float *theta, float *phi, float heldBank, float thrustToWeight);
    float bankSlewForwardTilt(float theta, float previous, bool rolling, float rate, float dt);
    float bankScaleTrimWithInflow(float trim, float inflow, float previousInflow);
    float bankLiftFromStick(float stick, float thrustToWeight);
    float bankThrustFromCollective(float collective, float thrustToWeight);
    float bankCollectiveFromThrust(float thrust, float thrustToWeight);
    float bankCalcCollective(float fz, float cosTilt, float thrustToWeight);
    float bankCollectiveTiltCos(float measuredTiltCos, float fz, float thrustToWeight);
    float bankCalcVerticalForce(float accX, float accY, float accZ, float pitchDeg, float rollDeg);
    float bankUpdateThrustTrim(float trim, float verticalForce, float modelVertical, bool measurable, float dt);
    float bankSlewHeadCheck(float offset, float target, float rate, float dt);
    void bankRotateTilt(float *theta, float *phi, float offset);
    void bankDragToTravelFrame(float dragX, float dragY, float offset, float *forward, float *lateral);
    void bankDragToLevelFrame(float dragX, float dragY, float pitch, float roll, float *forward, float *left);
    float bankTargetSpeed(float speedStick, float midSpeed);
    float bankMaxForwardForce(float fz, float thrustToWeight);
    float bankSpeedForce(float speedStick, float v, float midSpeed, float kp, float maxForce);
    float bankForceToTilt(float fx, float fz, float maxBrakeTilt);
    float bankCalcBall(float dragX, float dragY, float offset, float deadband, bool canReverse);
    bool bankCanReverse(float heldPitch);
    float bankScaleBallIntegrator(float bankTan, float previousBankTan, float *integrator);
    float bankTurnFeedforward(float bankTan, float speed, float turnGain);
    float bankCoordinatedBankLimit(float speed, float turnGain, float maxYawRate);
    float bankLearnTurnGain(float turnGain, float bankTan, float speed, float dt, float *integrator);
    float bankStepBallController(float ball, float kp, float ki, float maxRate, float dt, float *integrator);
    void bankCompute(const bankModeConfig_t *config, const bankInputs_t *in, bankState_t *state, bankOutputs_t *out);
    float bankLookIntoTurnOffset(float bankStick, float speedStick);
}

#include "gtest/gtest.h"

static const float DEG = M_PI / 180.0f;

// The yaw rate, deg/s, the nose offset (the look into the turn and the head check) slewed by in the last update
static float offsetSlewDps(float offsetBefore, float offsetAfter, float dt)
{
    return (offsetAfter - offsetBefore) / DEG / dt;
}

// The configured thrust-to-weight ratio, as a multiple of the weight.
static float defaultThrustToWeight(void)
{
    return bankModeConfig()->thrustToWeight / 100.0f;
}

// The collective that gives 1 g with the thrust curve thrust = T/W * collective ^ 1.5
static float hoverCollective(float thrustToWeight)
{
    return powf(1.0f / thrustToWeight, 1.0f / 1.5f);
}

TEST(BankModeTest, ConfigDefaults)
{
    pgResetAll();
    EXPECT_EQ(570, bankModeConfig()->thrustToWeight);
    EXPECT_EQ(90, bankModeConfig()->maxBankAngle);
    EXPECT_EQ(30, bankModeConfig()->maxBrakeAngle);
    EXPECT_EQ(600, bankModeConfig()->ballKp);
    EXPECT_EQ(600, bankModeConfig()->ballKi);
    EXPECT_EQ(20, bankModeConfig()->ballDeadband);
    EXPECT_EQ(200, bankModeConfig()->speedKp);
    EXPECT_EQ(20, bankModeConfig()->speedDeadband);
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

TEST(BankModeTest, ThrustCurveIsAPowerOfTheCollective)
{
    EXPECT_FLOAT_EQ(0.0f, bankThrustFromCollective(0.0f, 4.0f));
    EXPECT_NEAR(4.0f, bankThrustFromCollective(1.0f, 4.0f), 1e-6f);
    EXPECT_NEAR(4.0f * powf(0.5f, 1.5f), bankThrustFromCollective(0.5f, 4.0f), 1e-6f);
    for (float t = 0.0f; t <= 4.0f; t += 0.5f) {
        EXPECT_NEAR(t, bankThrustFromCollective(bankCollectiveFromThrust(t, 4.0f), 4.0f), 1e-5f);
    }
    EXPECT_FLOAT_EQ(1.0f, bankCollectiveFromThrust(9.0f, 4.0f));    // more than the maximum is the maximum
}

TEST(BankModeTest, CollectiveIsHoverAtCentreAndCompensatesTilt)
{
    const float tw = 2.0f;
    EXPECT_NEAR(1.0f, bankThrustFromCollective(bankCalcCollective(0.0f, cosf(0.0f) * cosf(0.0f), tw), tw), 1e-5f);
    // forward pitch with no vertical command raises thrust by 1 / cos(theta)
    const float theta = 30.0f * DEG;
    EXPECT_NEAR(1.0f / cosf(theta), bankThrustFromCollective(bankCalcCollective(0.0f, cosf(theta) * cosf(0.0f), tw), tw), 1e-5f);
    // bank and pitch together
    const float phi = 20.0f * DEG;
    const float both = bankThrustFromCollective(bankCalcCollective(0.0f, cosf(theta) * cosf(phi), tw), tw);
    EXPECT_NEAR(1.0f / (cosf(theta) * cosf(phi)), both, 1e-5f);
    // the vertical component of the thrust stays the weight: T * cos(tilt) == 1 + Fz
    const float t = bankThrustFromCollective(bankCalcCollective(0.2f, cosf(theta) * cosf(phi), tw), tw);
    EXPECT_NEAR(1.2f, t * cosf(theta) * cosf(phi), 1e-5f);
}

TEST(BankModeTest, CollectiveIsSetForTheMeasuredTilt)
{
    const float tw = 5.7f;
    // the bank commanded, the craft still level: the collective of a hover, not of the bank
    EXPECT_NEAR(1.0f, bankCollectiveTiltCos(1.0f, 0.0f, tw), 1e-6f);
    // still banked 60 degrees, the bank taken off: the collective that holds the weight at 60 degrees
    EXPECT_NEAR(0.5f, bankCollectiveTiltCos(0.5f, 0.0f, tw), 1e-6f);
    // tilted past what the thrust allows (an overshoot): no more than the thrust budget
    const float cosTiltMin = 1.0f / (1.0f + 0.75f * (tw - 1.0f));
    EXPECT_NEAR(cosTiltMin, bankCollectiveTiltCos(0.05f, 0.0f, tw), 1e-6f);
}

TEST(BankModeTest, TiltLimiterKeepsFzAndStaysWithinMaxThrust)
{
    const float tw = 1.5f;
    float theta = 40.0f * DEG;
    float phi = 40.0f * DEG;
    const float fz = bankLimitTilt(0.1f, &theta, &phi, 0.0f, tw);
    EXPECT_FLOAT_EQ(0.1f, fz);
    EXPECT_LE(bankCalcCollective(fz, cosf(theta) * cosf(phi), tw), 1.0f + 1e-5f);
    // the tilt uses its whole budget, which stops a quarter of the spare thrust short of the maximum
    const float tiltThrust = 1.1f + 0.75f * (tw - 1.1f);
    EXPECT_NEAR(tiltThrust, bankThrustFromCollective(bankCalcCollective(fz, cosf(theta) * cosf(phi), tw), tw), 1e-4f);
}

TEST(BankModeTest, TiltLimiterKeepsAReserveForTheVerticalLoop)
{
    // a 5 inch at the simulator's gravity, asked for a 90 degree bank at the centre of the lift stick
    const float tw = 4.8f;
    float theta = 0.0f;
    float phi = 90.0f * DEG;
    bankLimitTilt(0.0f, &theta, &phi, 0.0f, tw);
    const float tiltThrust = 1.0f + 0.75f * (tw - 1.0f);
    EXPECT_NEAR(acosf(1.0f / tiltThrust), phi, 1e-4f);                  // about 75 degrees
    EXPECT_LT(phi, acosf(1.0f / tw) - 2.0f * DEG);                      // short of the 78 the thrust could hold
    const float collective = bankCalcCollective(0.0f, cosf(theta) * cosf(phi), tw);
    EXPECT_LT(collective, 0.9f);                                        // room left to add lift
}

TEST(BankModeTest, TiltLimiterGivesBankPriorityOverForwardTilt)
{
    const float tw = 1.5f;
    float theta = 40.0f * DEG;
    float phi = 30.0f * DEG;
    bankLimitTilt(0.0f, &theta, &phi, 0.0f, tw);
    // bank alone fits the budget (cos 30 deg = 0.87 > 1 / 1.5), so it is untouched
    EXPECT_NEAR(30.0f * DEG, phi, 1e-5f);
    // forward tilt takes the rest, so it is trimmed
    EXPECT_LT(theta, 40.0f * DEG);
    EXPECT_GT(theta, 0.0f);
}

TEST(BankModeTest, TiltLimiterLeavesForwardTiltBesideTheBankTheCraftIsStillIn)
{
    const float tw = 5.7f;
    // the bank stick flicked through the centre while the craft is still banked at 75 degrees (either side)
    float theta = 70.0f * DEG;
    float phi = 0.0f;
    bankLimitTilt(0.0f, &theta, &phi, -75.0f * DEG, tw);
    float thetaLevel = 70.0f * DEG;
    float phiLevel = 0.0f;
    bankLimitTilt(0.0f, &thetaLevel, &phiLevel, 0.0f, tw);
    EXPECT_NEAR(0.0f, phi, 1e-6f);
    EXPECT_LT(theta, 35.0f * DEG);                                      // not the 70 degrees a level craft gets
    EXPECT_GT(thetaLevel, 60.0f * DEG);
    // and the bank asked for still counts when the craft is level
    float thetaRollIn = 70.0f * DEG;
    float phiRollIn = 75.0f * DEG;
    bankLimitTilt(0.0f, &thetaRollIn, &phiRollIn, 0.0f, tw);
    EXPECT_NEAR(theta, thetaRollIn, 1e-4f);
}

TEST(BankModeTest, ForwardTiltGrowsSlowlyNotAtAllWhileRollingAndFallsAtOnce)
{
    const float rate = 180.0f * DEG;
    EXPECT_NEAR(1.8f * DEG, bankSlewForwardTilt(60.0f * DEG, 0.0f, false, rate, 0.01f), 1e-6f);
    EXPECT_NEAR(31.8f * DEG, bankSlewForwardTilt(60.0f * DEG, 30.0f * DEG, false, rate, 0.01f), 1e-6f);
    EXPECT_FLOAT_EQ(30.0f * DEG, bankSlewForwardTilt(60.0f * DEG, 30.0f * DEG, true, rate, 0.01f));    // rolling: held
    EXPECT_FLOAT_EQ(5.0f * DEG, bankSlewForwardTilt(5.0f * DEG, 60.0f * DEG, true, rate, 0.01f));     // falls at once
    EXPECT_FLOAT_EQ(-30.0f * DEG, bankSlewForwardTilt(-30.0f * DEG, 20.0f * DEG, false, rate, 0.01f)); // brake at once
    EXPECT_NEAR(1.8f * DEG, bankSlewForwardTilt(60.0f * DEG, -30.0f * DEG, false, rate, 0.01f), 1e-6f); // from a brake
}

TEST(BankModeTest, TiltLimiterTrimsBankOnlyWhenItAloneExceedsBudget)
{
    const float tw = 1.2f;
    float theta = 30.0f * DEG;
    float phi = 45.0f * DEG;
    bankLimitTilt(0.0f, &theta, &phi, 0.0f, tw);
    // the tilt may use 1 + 0.75 * 0.2 = 1.15 g, so cos >= 1 / 1.15: 29.6 deg, and nothing is left for pitch
    EXPECT_NEAR(acosf(1.0f / 1.15f), phi, 1e-4f);
    EXPECT_NEAR(0.0f, theta, 1e-3f);
}

TEST(BankModeTest, TiltLimiterKeepsSigns)
{
    float theta = -40.0f * DEG;
    float phi = -30.0f * DEG;
    bankLimitTilt(0.0f, &theta, &phi, 0.0f, 1.5f);
    EXPECT_LT(theta, 0.0f);
    EXPECT_LT(phi, 0.0f);
}

TEST(BankModeTest, TiltLimiterCapsFzWhenThrustCannotLiftIt)
{
    float theta = 20.0f * DEG;
    float phi = 20.0f * DEG;
    const float fz = bankLimitTilt(2.0f, &theta, &phi, 0.0f, 1.5f);
    EXPECT_FLOAT_EQ(0.5f, fz);
    EXPECT_NEAR(0.0f, theta, 1e-3f);
    EXPECT_NEAR(0.0f, phi, 1e-3f);
}

TEST(BankModeTest, TiltLimiterAllowsFullDescent)
{
    float theta = 20.0f * DEG;
    float phi = 20.0f * DEG;
    const float fz = bankLimitTilt(-2.0f, &theta, &phi, 0.0f, 2.0f);
    EXPECT_FLOAT_EQ(-1.0f, fz);
    EXPECT_FLOAT_EQ(0.0f, bankCalcCollective(fz, cosf(theta) * cosf(phi), 2.0f));
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
    const float before = bankCalcCollective(0.1f, cosf(theta0) * cosf(phi0), tw);
    for (float offsetDeg = -120.0f; offsetDeg <= 120.0f; offsetDeg += 30.0f) {
        float theta = theta0;
        float phi = phi0;
        bankRotateTilt(&theta, &phi, offsetDeg * DEG);
        EXPECT_NEAR(cosf(theta0) * cosf(phi0), cosf(theta) * cosf(phi), 1e-5f);
        EXPECT_NEAR(before, bankCalcCollective(0.1f, cosf(theta) * cosf(phi), tw), 1e-5f);
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

TEST(BankModeTest, LookIntoTurnOffsetComesFromBankAgainstSpeed)
{
    // no bank: along the track, whatever the speed
    EXPECT_FLOAT_EQ(0.0f, bankLookIntoTurnOffset(0.0f, 0.0f));
    EXPECT_FLOAT_EQ(0.0f, bankLookIntoTurnOffset(0.0f, 1.0f));
    // a big bank at no speed: a quarter turn, clockwise for a right bank
    EXPECT_NEAR(-90.0f * DEG, bankLookIntoTurnOffset(1.0f, 0.0f), 1e-5f);
    EXPECT_NEAR(90.0f * DEG, bankLookIntoTurnOffset(-1.0f, 0.0f), 1e-5f);
    // full bank at full speed: half way
    EXPECT_NEAR(-45.0f * DEG, bankLookIntoTurnOffset(1.0f, 1.0f), 1e-5f);
    // a little bank at full speed: a little
    EXPECT_NEAR(-atan2f(0.2f, 1.0f), bankLookIntoTurnOffset(0.2f, 1.0f), 1e-5f);
    // more bank against the same speed looks further in
    EXPECT_LT(bankLookIntoTurnOffset(0.8f, 0.5f), bankLookIntoTurnOffset(0.4f, 0.5f));
    // a small bank at no speed fades in instead of swinging the nose round
    EXPECT_NEAR(-45.0f * DEG, bankLookIntoTurnOffset(0.1f, 0.0f), 1e-5f);
}

TEST(BankModeTest, BallIsCentredWithNoSlip)
{
    // travelling straight ahead: the drag points straight back, and there is nothing to correct
    EXPECT_FLOAT_EQ(0.0f, bankCalcBall(-0.2f, 0.0f, 0.0f, 0.0f, true));
    EXPECT_FLOAT_EQ(0.0f, bankCalcBall(0.0f, 0.0f, 0.0f, 0.0f, true));
}

TEST(BankModeTest, BallAsksTheNoseToFollowTheSlip)
{
    // drifting to the left: the drag points to the right (-Y), so the nose should turn left (positive)
    EXPECT_GT(bankCalcBall(-0.2f, -0.1f, 0.0f, 0.0f, true), 0.0f);
    // drifting to the right: nose turns right (negative)
    EXPECT_LT(bankCalcBall(-0.2f, 0.1f, 0.0f, 0.0f, true), 0.0f);
    // sideways from rest still turns towards the slip
    EXPECT_GT(bankCalcBall(0.0f, -0.1f, 0.0f, 0.0f, true), 0.0f);
}

TEST(BankModeTest, BallIsProportionalToTheLateralDragWithNoSpeedFade)
{
    EXPECT_NEAR(0.1f, bankCalcBall(-0.2f, -0.1f, 0.0f, 0.0f, true), 1e-6f);
    EXPECT_NEAR(0.2f, bankCalcBall(-0.2f, -0.2f, 0.0f, 0.0f, true), 1e-6f);
    // the forward drag does not change it
    EXPECT_NEAR(0.1f, bankCalcBall(-0.6f, -0.1f, 0.0f, 0.0f, true), 1e-6f);
}

TEST(BankModeTest, BallDeadbandIsSubtractive)
{
    EXPECT_FLOAT_EQ(0.0f, bankCalcBall(-0.2f, -0.02f, 0.0f, 0.03f, true));
    EXPECT_FLOAT_EQ(0.0f, bankCalcBall(-0.2f, 0.03f, 0.0f, 0.03f, true));
    EXPECT_NEAR(0.02f, bankCalcBall(-0.2f, -0.05f, 0.0f, 0.03f, true), 1e-6f);
    EXPECT_NEAR(-0.02f, bankCalcBall(-0.2f, 0.05f, 0.0f, 0.03f, true), 1e-6f);
}

TEST(BankModeTest, BallFlipsSignWhenReversingSoTheTailFollows)
{
    // moving backwards: the drag points forward (+X). Drifting to the left, the tail moves left,
    // so the nose turns right (negative)
    EXPECT_LT(bankCalcBall(0.2f, -0.1f, 0.0f, 0.02f, true), 0.0f);
    EXPECT_GT(bankCalcBall(0.2f, 0.1f, 0.0f, 0.02f, true), 0.0f);
    // it goes through zero between clearly forward and clearly backward travel, and is continuous
    float last = bankCalcBall(-0.2f, -0.1f, 0.0f, 0.02f, true);
    for (float x = -0.2f; x <= 0.2f; x += 0.005f) {
        const float ball = bankCalcBall(x, -0.1f, 0.0f, 0.02f, true);
        EXPECT_LT(fabsf(ball - last), 0.05f);
        last = ball;
    }
}

TEST(BankModeTest, BallIsMeasuredFromTheHeadCheckFrame)
{
    // travelling straight ahead with the nose already at the offset (the travel is 30 degrees to the
    // right of the nose): the ball is centred, but it would not be without the offset
    const float beta = -30.0f * DEG;
    const float dragX = -0.2f * cosf(beta);
    const float dragY = -0.2f * sinf(beta);
    EXPECT_NEAR(0.0f, bankCalcBall(dragX, dragY, 30.0f * DEG, 0.0f, true), 1e-6f);
    EXPECT_LT(bankCalcBall(dragX, dragY, 0.0f, 0.0f, true), 0.0f);
}

TEST(BankModeTest, TurnFeedforwardIsTheCoordinatedRateAndFadesOutAtRest)
{
    // 45 degrees to the right at 0.4 g of drag: g * tan(phi) / v, clockwise (negative)
    EXPECT_NEAR(-19.0f / 0.4f, bankTurnFeedforward(1.0f, 0.4f, 19.0f), 1e-4f);
    EXPECT_NEAR(-2.0f * 19.0f / 0.4f, bankTurnFeedforward(2.0f, 0.4f, 19.0f), 1e-4f);     // tan(phi) twice as big
    EXPECT_NEAR(-19.0f / 0.8f, bankTurnFeedforward(1.0f, 0.8f, 19.0f), 1e-4f);            // twice as fast: half the rate
    EXPECT_FLOAT_EQ(0.0f, bankTurnFeedforward(1.0f, 0.0f, 19.0f));                         // at rest the nose stays put
    EXPECT_LT(fabsf(bankTurnFeedforward(1.0f, 0.05f, 19.0f)), fabsf(bankTurnFeedforward(1.0f, 0.1f, 19.0f)));
    EXPECT_GT(bankTurnFeedforward(1.0f, -0.4f, 19.0f), 0.0f);                              // reversing: the other way
    EXPECT_FLOAT_EQ(0.0f, bankTurnFeedforward(0.0f, 0.4f, 19.0f));
}

TEST(BankModeTest, TurnGainLearnsFromTheBallIntegratorWithoutAStep)
{
    // the ball integrator holds another 10 deg/s clockwise: the gain is too low
    float integrator = -10.0f;
    float gain = 19.0f;
    const float before = bankTurnFeedforward(1.0f, 0.4f, gain) + integrator;
    gain = bankLearnTurnGain(gain, 1.0f, 0.4f, 0.01f, &integrator);
    EXPECT_GT(gain, 19.0f);
    EXPECT_NEAR(before, bankTurnFeedforward(1.0f, 0.4f, gain) + integrator, 1e-3f);
    for (int i = 0; i < 2000; i++) {
        gain = bankLearnTurnGain(gain, 1.0f, 0.4f, 0.01f, &integrator);
    }
    EXPECT_NEAR(0.0f, integrator, 1e-3f);
    EXPECT_NEAR(19.0f + 10.0f * 0.4f, gain, 1e-2f);
    // not learnt flying straight or slowly
    integrator = -10.0f;
    EXPECT_FLOAT_EQ(19.0f, bankLearnTurnGain(19.0f, 0.0f, 0.4f, 0.01f, &integrator));
    EXPECT_FLOAT_EQ(19.0f, bankLearnTurnGain(19.0f, 1.0f, 0.05f, 0.01f, &integrator));
    EXPECT_FLOAT_EQ(-10.0f, integrator);
}

TEST(BankModeTest, BallControllerProportionalAndIntegral)
{
    float integrator = 0.0f;
    // one second at a ball of 0.1: P = 300 * 0.1 = 30, I rises by 300 * 0.1 = 30 per second
    float rate = 0.0f;
    for (int i = 0; i < 100; i++) {
        rate = bankStepBallController(0.1f, 300.0f, 300.0f, 360.0f, 0.01f, &integrator);
    }
    EXPECT_NEAR(30.0f, integrator, 1e-2f);
    EXPECT_NEAR(60.0f, rate, 1e-2f);
    // the centred ball keeps the integrator, which is the rate a steady turn needs
    rate = bankStepBallController(0.0f, 300.0f, 300.0f, 360.0f, 0.01f, &integrator);
    EXPECT_NEAR(30.0f, rate, 1e-2f);
}

TEST(BankModeTest, BallControllerIsLimited)
{
    float integrator = 0.0f;
    float rate = 0.0f;
    for (int i = 0; i < 10000; i++) {
        rate = bankStepBallController(1.0f, 300.0f, 300.0f, 360.0f, 0.01f, &integrator);
    }
    EXPECT_FLOAT_EQ(360.0f, rate);
    EXPECT_FLOAT_EQ(360.0f, integrator);
    // no wind-up: the rate unwinds as soon as the ball reverses
    for (int i = 0; i < 300; i++) {
        rate = bankStepBallController(-1.0f, 300.0f, 300.0f, 360.0f, 0.01f, &integrator);
    }
    EXPECT_LT(rate, 0.0f);
}

// Point mass with rotor drag, flying a bank with the yaw driven only by the ball. The controller is
// never told the speed. The craft holds a constant forward force that balances the drag at v0.
static void flyBankedTurn(float v0, float bankDeg, float *finalOmega, float *finalBall, float *finalSpeed)
{
    const float g = 9.81f;
    const float k = 0.3f;                           // rotor drag, 1/s
    const float dt = 0.01f;
    const float phi = bankDeg * DEG;
    float vx = v0;                                  // velocity in the body frame
    float vy = 0.0f;
    float omega = 0.0f;                             // counter-clockwise, rad/s
    float integrator = 0.0f;
    float ball = 0.0f;

    for (int i = 0; i < 6000; i++) {
        const float dragX = -k * vx / g;
        const float dragY = -k * vy / g;
        ball = bankCalcBall(dragX, dragY, 0.0f, 0.0f, true);
        omega = bankStepBallController(ball, 300.0f, 300.0f, 360.0f, dt, &integrator) * DEG;

        // thrust: the bank tilts it to the right (-Y) with g * tan(phi); the forward force holds the speed
        const float ax = k * v0 - k * vx + omega * vy;
        const float ay = -g * tanf(phi) - k * vy - omega * vx;
        vx += ax * dt;
        vy += ay * dt;
    }
    *finalOmega = omega;
    *finalBall = ball;
    *finalSpeed = sqrtf(vx * vx + vy * vy);
}

TEST(BankModeTest, BallLoopSettlesIntoACoordinatedTurnAtAnySpeed)
{
    // omega = g * tan(phi) / v, to the right (negative), found without the controller knowing v
    const float speeds[] = {4.0f, 8.0f, 15.0f};
    const float banks[] = {15.0f, 30.0f, 45.0f};
    for (float v0 : speeds) {
        for (float bank : banks) {
            float omega, ball, speed;
            flyBankedTurn(v0, bank, &omega, &ball, &speed);
            const float expected = -9.81f * tanf(bank * DEG) / speed;
            EXPECT_NEAR(0.0f, ball, 1e-3f) << "v0 " << v0 << " bank " << bank;
            EXPECT_NEAR(expected, omega, fabsf(expected) * 0.05f) << "v0 " << v0 << " bank " << bank;
        }
    }
}

static bankInputs_t centredInputs(void)
{
    // speed stick at the bottom (stopped), lift stick at the centre (1 g of lift, hover)
    bankInputs_t in = {};
    in.lift = 0.5f;
    in.dt = 0.01f;
    return in;
}

// Flies the bank stick for steps updates at 0.48 g of drag (about 19 m/s), with the craft holding the attitude it is
// asked for and the drag seen from the nose: travel-frame drag forward and lateral, rotated by the nose offset.
static void flySteadyBank(const bankModeConfig_t *config, bankInputs_t *in, bankState_t *state, bankOutputs_t *out,
    float lateralDrag, int steps)
{
    for (int i = 0; i < steps; i++) {
        const float c = cosf(state->headCheckOffset);
        const float s = sinf(state->headCheckOffset);
        in->dragX = -0.5f * c + lateralDrag * s;
        in->dragY = 0.5f * s + lateralDrag * c;
        bankCompute(config, in, state, out);
        in->measuredPitchDeg = out->pitchDeg;
        in->measuredRollDeg = out->rollDeg;
    }
}

static bankInputs_t steadyBankInputs(void)
{
    bankInputs_t in = centredInputs();
    in.speed = 0.5f;
    in.roll = 1.0f;                     // a hard right bank
    return in;
}

TEST(BankModeTest, ACraftTiltedForwardIsNotReversingWhateverTheDragSays)
{
    EXPECT_TRUE(bankCanReverse(0.0f));
    EXPECT_TRUE(bankCanReverse(-30.0f * DEG));                          // tilted back after a brake
    EXPECT_FALSE(bankCanReverse(30.0f * DEG));
    // rearward drag tilted forward (a fast climb): the ball keeps its forward sense
    EXPECT_GT(bankCalcBall(0.2f, -0.1f, 0.0f, 0.02f, false), 0.0f);
    EXPECT_LT(bankCalcBall(0.2f, -0.1f, 0.0f, 0.02f, true), 0.0f);
    // and the speed reads no less than 0, so the turn is not fed forward the wrong way
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.speed = 0.75f;
    in.dragX = 0.5f;
    in.measuredPitchDeg = 50.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_GE(state.speed, 0.0f);
    in.measuredPitchDeg = 0.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_LT(state.speed, 0.0f);
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
    EXPECT_NEAR(hoverCollective(defaultThrustToWeight()), out.collective, 1e-5f);
}

TEST(BankModeTest, ComputeSetsTheCollectiveForTheMeasuredTiltNotTheTarget)
{
    pgResetAll();
    bankModeConfig_t config = *bankModeConfig();
    config.stickDeadband = 0;
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.roll = 1.0f;                     // full bank, from the hover
    bankCompute(&config, &in, &state, &out);
    EXPECT_GT(out.rollDeg, 25.0f);
    EXPECT_NEAR(hoverCollective(defaultThrustToWeight()), out.collective, 1e-5f);     // still level
    in.measuredRollDeg = out.rollDeg;   // the craft has rolled in
    bankCompute(&config, &in, &state, &out);
    const float cosTilt = cosf(out.rollDeg * DEG);
    EXPECT_NEAR(1.0f / cosTilt, bankThrustFromCollective(out.collective, defaultThrustToWeight()), 0.02f);
}

TEST(BankModeTest, ComputeSignsFollowTheSticks)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.roll = 1.0f;
    in.speed = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    // seen from the track: bank right, and no forward tilt while the craft rolls in
    float theta = out.pitchDeg * DEG;
    float phi = out.rollDeg * DEG;
    bankRotateTilt(&theta, &phi, -state.headCheckOffset);
    EXPECT_GT(phi, 0.0f);
    EXPECT_NEAR(0.0f, theta, 1e-5f);
    EXPECT_LT(state.headCheckOffset, 0.0f);    // the nose turns right, into the turn
    for (int i = 0; i < 100; i++) {
        in.measuredRollDeg = out.rollDeg;
        in.measuredPitchDeg = out.pitchDeg;
        bankCompute(bankModeConfig(), &in, &state, &out);
    }
    theta = out.pitchDeg * DEG;
    phi = out.rollDeg * DEG;
    bankRotateTilt(&theta, &phi, -state.headCheckOffset);
    EXPECT_GT(theta, 0.0f);         // forward
    // once the craft has tilted, the tilt adds collective so the vertical force is unchanged
    in.measuredRollDeg = out.rollDeg;
    in.measuredPitchDeg = out.pitchDeg;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_GT(out.collective, hoverCollective(defaultThrustToWeight()) + 0.01f);
}

TEST(BankModeTest, BankIsLimitedToATurnTheYawCanFollow)
{
    // turn gain 19 deg/s g, yaw limit 360 deg/s: 0.8 * 360 * speed / 19 is the steepest tan(bank)
    EXPECT_NEAR(30.0f * DEG, bankCoordinatedBankLimit(0.0f, 19.0f, 360.0f), 1e-5f);      // at rest, the floor
    EXPECT_NEAR(atanf(0.8f * 360.0f * 0.2f / 19.0f), bankCoordinatedBankLimit(0.2f, 19.0f, 360.0f), 1e-5f);
    EXPECT_GT(bankCoordinatedBankLimit(0.5f, 19.0f, 360.0f), 80.0f * DEG);              // fast: thrust limits it first
    EXPECT_FLOAT_EQ(bankCoordinatedBankLimit(0.2f, 19.0f, 360.0f), bankCoordinatedBankLimit(-0.2f, 19.0f, 360.0f));
    // a faster yaw allows a steeper bank at the same speed
    EXPECT_GT(bankCoordinatedBankLimit(0.1f, 19.0f, 720.0f), bankCoordinatedBankLimit(0.1f, 19.0f, 360.0f));
}

TEST(BankModeTest, LiftIsNoneAtTheBottomOneGAtTheCentreAndMaxThrustAtTheTop)
{
    for (float tw = 1.1f; tw <= 6.0f; tw += 0.7f) {
        EXPECT_FLOAT_EQ(0.0f, bankLiftFromStick(0.0f, tw));
        EXPECT_NEAR(1.0f, bankLiftFromStick(0.5f, tw), 1e-6f);
        EXPECT_NEAR(tw, bankLiftFromStick(1.0f, tw), 1e-5f);
        // linear below the centre, a cubic above it that starts with the same slope (1 g per half stick)
        EXPECT_NEAR(0.5f, bankLiftFromStick(0.25f, tw), 1e-6f);
        if (tw > 2.0f) {
            EXPECT_NEAR(1.0f + 0.5f + (tw - 2.0f) * 0.125f, bankLiftFromStick(0.75f, tw), 1e-5f);
        }
        if (tw > 2.0f) {
            EXPECT_NEAR(1.0f + 0.02f, bankLiftFromStick(0.51f, tw), 0.001f);   // 1 g per half stick, as below
        }
        float last = 0.0f;
        for (float stick = 0.0f; stick <= 1.0f; stick += 0.01f) {
            EXPECT_GE(bankLiftFromStick(stick, tw), last);
            last = bankLiftFromStick(stick, tw);
        }
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
        in.lift = 1.0f;
        in.roll = 1.0f;
        in.speed = 1.0f;
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

    in.lift = 0.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(0.0f, out.collective);          // no lift, the craft drops with gravity

    in.lift = 0.5f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_NEAR(hoverCollective(defaultThrustToWeight()), out.collective, 1e-5f);       // thrust equals weight

    in.lift = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_NEAR(1.0f, out.collective, 1e-5f);       // 2 g of lift at T/W = 2
}

TEST(BankModeTest, ComputeThrustIsLinearInLiftBelowTheCentre)
{
    pgResetAll();
    bankModeConfig_t config = *bankModeConfig();
    config.stickDeadband = 0;
    bankState_t state = {};
    bankOutputs_t a = {};
    bankOutputs_t b = {};
    bankInputs_t in = centredInputs();
    in.lift = 0.25f;
    bankCompute(&config, &in, &state, &a);
    in.lift = 0.5f;
    bankCompute(&config, &in, &state, &b);
    const float tw = defaultThrustToWeight();
    EXPECT_NEAR(bankThrustFromCollective(b.collective, tw), 2.0f * bankThrustFromCollective(a.collective, tw), 1e-5f);
}

TEST(BankModeTest, ComputeBottomStickWithTiltStillGivesNoLift)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.lift = 0.0f;
    in.roll = 1.0f;
    in.speed = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(0.0f, out.collective);
}

TEST(BankModeTest, ComputeSticksInsideTheDeadbandMeanNoTilt)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.roll = 0.04f;    // inside the deadband
    in.speed = 0.04f;   // just above the bottom
    in.lift = 0.5f + 0.02f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(0.0f, out.rollDeg);
    EXPECT_FLOAT_EQ(0.0f, out.pitchDeg);
    EXPECT_NEAR(hoverCollective(defaultThrustToWeight()), out.collective, 1e-5f);   // the hover is not disturbed either
}

TEST(BankModeTest, EngagingOnTheGroundWithTheRollStickOffCentreHoldsTiltUntilCentred)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.throttleLow = true;
    in.roll = 1.0f;
    in.speed = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(0.0f, out.rollDeg);
    EXPECT_FLOAT_EQ(0.0f, out.pitchDeg);

    // still held while the stick stays deflected, even once the throttle is raised
    in.throttleLow = false;
    in.lift = 0.5f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(0.0f, out.rollDeg);

    // centring releases it
    in.roll = 0.0f;
    in.speed = 0.0f;
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

TEST(BankModeTest, ComputeSpeedStickAtTheBottomKeepsTheHoverCollective)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.throttleLow = true;      // the left stick at the bottom is zero speed, not zero lift
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_NEAR(hoverCollective(defaultThrustToWeight()), out.collective, 1e-5f);
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
            in.speed = 1.0f;
            in.lift = vertical;
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
    const float offsetBefore = state.headCheckOffset;
    in.yaw = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_NEAR(collectiveBefore, out.collective, 1e-5f);
    // with a centred ball only the slew rate drives the nose, in the stick direction, from where the look into the
    // bank has it
    EXPECT_GT(out.yawRateDps, 0.0f);
    EXPECT_GT(state.headCheckOffset, offsetBefore);
}

TEST(BankModeTest, ComputeYawFollowsTheBallAndIntegratorResetsWithNoLift)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.dragX = -0.3f;
    in.dragY = -0.2f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_GT(out.yawRateDps, 0.0f);
    EXPECT_GT(state.ballIntegrator, 0.0f);

    in.dragY = 0.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_GT(out.yawRateDps, 0.0f);   // the integrator still holds the turn rate

    in.lift = 0.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(0.0f, state.ballIntegrator);
    EXPECT_FLOAT_EQ(0.0f, out.yawRateDps);
}

TEST(BankModeTest, BallIntegratorShrinksWithTheBank)
{
    float integrator = -30.0f;
    // bank held or increased: left to the ball
    EXPECT_FLOAT_EQ(0.5f, bankScaleBallIntegrator(0.5f, 0.5f, &integrator));
    EXPECT_FLOAT_EQ(-30.0f, integrator);
    bankScaleBallIntegrator(0.8f, 0.5f, &integrator);
    EXPECT_FLOAT_EQ(-30.0f, integrator);
    // halved: the turn needs half the rate
    bankScaleBallIntegrator(0.4f, 0.8f, &integrator);
    EXPECT_NEAR(-15.0f, integrator, 1e-5f);
    // centred, or reversed to the other side: nothing of the old turn is kept
    bankScaleBallIntegrator(0.0f, 0.4f, &integrator);
    EXPECT_FLOAT_EQ(0.0f, integrator);
    integrator = -30.0f;
    bankScaleBallIntegrator(-0.4f, 0.4f, &integrator);
    EXPECT_FLOAT_EQ(0.0f, integrator);
    // straight flight: the ball's trim is left alone, with no division by zero
    integrator = 2.0f;
    bankScaleBallIntegrator(0.0f, 0.0f, &integrator);
    EXPECT_FLOAT_EQ(2.0f, integrator);
}

TEST(BankModeTest, ComputeCentringTheBankStickClearsTheTurnRate)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = steadyBankInputs();
    in.roll = -0.6f;                    // banked left, the ball asks for a left turn
    flySteadyBank(bankModeConfig(), &in, &state, &out, -0.1f, 100);
    EXPECT_GT(state.ballIntegrator, 10.0f);
    in.roll = 0.0f;                     // rolled out, flying straight
    in.measuredPitchDeg = 0.0f;
    in.measuredRollDeg = 0.0f;
    const float offsetBefore = state.headCheckOffset;
    flySteadyBank(bankModeConfig(), &in, &state, &out, 0.0f, 1);
    EXPECT_FLOAT_EQ(0.0f, state.ballIntegrator);
    // only the look going back to the track turns the nose
    EXPECT_NEAR(offsetSlewDps(offsetBefore, state.headCheckOffset, in.dt), out.yawRateDps, 1e-3f);
}

TEST(BankModeTest, ComputeTurnRateFollowsTheBankTheCraftIsInNotTheStick)
{
    // the bank stick flicked from a hard right bank to a hard left one: the craft is still banked right and still
    // turning right, so the turn fed forward keeps turning the nose right until the craft has rolled through (the look
    // into the turn swings it left on top, ComputeLookSwingsWithTheBankStickOnAReversal), and the ball is not integrated
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.speed = 0.5f;
    in.dragX = -0.5f;
    in.roll = -1.0f;
    in.measuredRollDeg = 60.0f;
    in.dragY = 0.2f;                    // a big ball while rolling
    bankCompute(bankModeConfig(), &in, &state, &out);
    float theta = out.pitchDeg * DEG;
    float phi = out.rollDeg * DEG;
    bankRotateTilt(&theta, &phi, -state.headCheckOffset);
    EXPECT_LT(phi, -60.0f * DEG);       // the bank asked for is hard left
    EXPECT_LT(out.yawRateDps - offsetSlewDps(0.0f, state.headCheckOffset, in.dt), -20.0f);  // the turn is still right (clockwise)
    EXPECT_FLOAT_EQ(0.0f, state.ballIntegrator);
    // through level, no turn is fed forward
    in.measuredRollDeg = 0.0f;
    in.dragX = -0.5f * cosf(state.headCheckOffset);   // straight along the track, seen from the nose
    in.dragY = 0.5f * sinf(state.headCheckOffset);
    const float offsetBefore = state.headCheckOffset;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_NEAR(offsetSlewDps(offsetBefore, state.headCheckOffset, in.dt), out.yawRateDps, 1e-3f);
}

TEST(BankModeTest, ComputeBallIsNotIntegratedWhileTheHeadCheckSlews)
{
    pgResetAll();
    bankModeConfig_t config = *bankModeConfig();
    config.ballDeadband = 0;
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.dragX = -0.3f;                   // flying straight ahead, the nose still on the track
    in.yaw = 1.0f;                      // head check: the offset slews away, the nose lags behind it
    bankCompute(&config, &in, &state, &out);
    EXPECT_NE(0.0f, state.headCheckOffset);
    EXPECT_FLOAT_EQ(0.0f, state.ballIntegrator);
    EXPECT_GT(out.yawRateDps, 0.0f);    // the slew still turns the nose
    // the offset has arrived: the ball is integrated again
    state.headCheckOffset = DEGREES_TO_RADIANS(config.headCheckRange);
    bankCompute(&config, &in, &state, &out);
    EXPECT_NE(0.0f, state.ballIntegrator);
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

// The look the sticks of in ask for, as bankCompute() shapes them
static float stickLook(const bankModeConfig_t *config, const bankInputs_t *in)
{
    const float deadband = config->stickDeadband / 100.0f;
    return bankLookIntoTurnOffset(bankShapeStick(in->roll, deadband), bankShapeStick(in->speed, deadband));
}

TEST(BankModeTest, ComputeLookIntoTheTurnFollowsTheSticks)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = steadyBankInputs();
    // the bank is still asked for in the travel frame, as a car on a banked track; the nose slews into the turn
    flySteadyBank(bankModeConfig(), &in, &state, &out, 0.0f, 1);
    EXPECT_NEAR(-bankModeConfig()->headCheckRate * DEG * in.dt, state.headCheckOffset, 1e-5f);
    flySteadyBank(bankModeConfig(), &in, &state, &out, 0.0f, 300);
    EXPECT_NEAR(stickLook(bankModeConfig(), &in), state.headCheckOffset, 1e-4f);
    EXPECT_LT(state.headCheckOffset, -45.0f * DEG);    // full bank against half speed: more than half way in
    // and the track is unchanged: seen from the track, the tilt is the bank asked for
    float theta = out.pitchDeg * DEG;
    float phi = out.rollDeg * DEG;
    bankRotateTilt(&theta, &phi, -state.headCheckOffset);
    EXPECT_GT(phi, 45.0f * DEG);
    // more speed, less look
    const float offsetHalfSpeed = state.headCheckOffset;
    in.speed = 1.0f;
    flySteadyBank(bankModeConfig(), &in, &state, &out, 0.0f, 300);
    EXPECT_GT(state.headCheckOffset, offsetHalfSpeed);
    EXPECT_NEAR(-45.0f * DEG, state.headCheckOffset, 1e-4f);
}

TEST(BankModeTest, ComputeHeadCheckAddsToTheLookIntoTheTurn)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = steadyBankInputs();
    in.yaw = 1.0f;                      // head check to the left, out of the right turn
    flySteadyBank(bankModeConfig(), &in, &state, &out, 0.0f, 300);
    EXPECT_NEAR(stickLook(bankModeConfig(), &in) + bankModeConfig()->headCheckRange * DEG, state.headCheckOffset, 1e-4f);
}

TEST(BankModeTest, ComputeBallIsIntegratedInASteadyTurnWhileTheLookFollowsTheTilt)
{
    pgResetAll();
    bankModeConfig_t config = *bankModeConfig();
    config.ballDeadband = 0;
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = steadyBankInputs();
    in.roll = 0.6f;
    flySteadyBank(&config, &in, &state, &out, 0.0f, 300);
    // a slip, with the speed stick moving slowly so the look moves a little on every update
    state.ballIntegrator = 0.0f;
    for (int i = 0; i < 50; i++) {
        in.speed = 0.5f + 0.002f * i;
        const float previousOffset = state.headCheckOffset;
        flySteadyBank(&config, &in, &state, &out, -0.05f, 1);
        ASSERT_LT(fabsf(state.headCheckOffset - previousOffset), config.headCheckRate * DEG * in.dt);
    }
    EXPECT_GT(state.ballIntegrator, 0.0f);
}

TEST(BankModeTest, ComputeLookSwingsWithTheBankStickOnAReversal)
{
    // the bank stick flicked from a hard right bank to a hard left one: the look turns to the new side at once, at the
    // head check rate
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = steadyBankInputs();
    flySteadyBank(bankModeConfig(), &in, &state, &out, 0.0f, 300);
    const float offsetBefore = state.headCheckOffset;
    in.roll = -1.0f;
    flySteadyBank(bankModeConfig(), &in, &state, &out, 0.0f, 1);
    EXPECT_NEAR(offsetBefore + bankModeConfig()->headCheckRate * DEG * in.dt, state.headCheckOffset, 1e-4f);
    flySteadyBank(bankModeConfig(), &in, &state, &out, 0.0f, 300);
    EXPECT_NEAR(stickLook(bankModeConfig(), &in), state.headCheckOffset, 1e-4f);
    EXPECT_GT(state.headCheckOffset, 45.0f * DEG);
}

TEST(BankModeTest, DragToTravelFrameRotatesByTheOffset)
{
    float forward, lateral;
    bankDragToTravelFrame(-0.3f, 0.0f, 0.0f, &forward, &lateral);
    EXPECT_NEAR(-0.3f, forward, 1e-6f);
    EXPECT_NEAR(0.0f, lateral, 1e-6f);
    // nose a quarter turn counter-clockwise from the travel direction: body Y is the travel direction's X
    bankDragToTravelFrame(0.0f, 0.3f, 90.0f * DEG, &forward, &lateral);
    EXPECT_NEAR(-0.3f, forward, 1e-5f);
    EXPECT_NEAR(0.0f, lateral, 1e-5f);
}

TEST(BankModeTest, DragToTravelFrameIsTheDragItselfWithNoOffset)
{
    float forward, lateral;
    bankDragToTravelFrame(-0.3f, 0.1f, 0.0f, &forward, &lateral);
    EXPECT_FLOAT_EQ(-0.3f, forward);
    EXPECT_FLOAT_EQ(0.1f, lateral);
}

// The in-plane rotor drag of a craft flying horizontally at (forward, left) drag units along the nose's heading.
static void inPlaneDrag(float forward, float left, float pitch, float roll, float *dragX, float *dragY)
{
    // the heading's forward and left axes, in the body frame (X forward, Y left), pitch nose-down, roll right
    *dragX = forward * cosf(pitch);
    *dragY = forward * sinf(roll) * sinf(pitch) + left * cosf(roll);
}

TEST(BankModeTest, LevelFrameUndoesTheTiltOfTheRotorPlane)
{
    float dragX, dragY, forward, left;
    bankDragToLevelFrame(-0.3f, 0.1f, 0.0f, 0.0f, &forward, &left);
    EXPECT_NEAR(-0.3f, forward, 1e-6f);
    EXPECT_NEAR(0.1f, left, 1e-6f);
    // pitched only: the body sees cos(pitch) of the forward drag
    inPlaneDrag(-0.3f, 0.0f, 30.0f * DEG, 0.0f, &dragX, &dragY);
    bankDragToLevelFrame(dragX, dragY, 30.0f * DEG, 0.0f, &forward, &left);
    EXPECT_NEAR(-0.3f, forward, 1e-5f);
    EXPECT_NEAR(0.0f, left, 1e-5f);
    // pitched and banked in a turn: part of the forward drag lands on body Y, but it is no slip
    inPlaneDrag(-0.3f, 0.0f, 10.0f * DEG, 26.0f * DEG, &dragX, &dragY);
    EXPECT_GT(fabsf(dragY), 0.02f);
    bankDragToLevelFrame(dragX, dragY, 10.0f * DEG, 26.0f * DEG, &forward, &left);
    EXPECT_NEAR(-0.3f, forward, 1e-5f);
    EXPECT_NEAR(0.0f, left, 1e-5f);
    // a real slip is still seen, at full size
    inPlaneDrag(-0.3f, -0.1f, 10.0f * DEG, 26.0f * DEG, &dragX, &dragY);
    bankDragToLevelFrame(dragX, dragY, 10.0f * DEG, 26.0f * DEG, &forward, &left);
    EXPECT_NEAR(-0.1f, left, 1e-5f);
}

TEST(BankModeTest, ComputeBallIsCentredInAPitchedBankedTurnWithNoSlip)
{
    pgResetAll();
    bankModeConfig_t config = *bankModeConfig();
    config.ballDeadband = 0;
    config.stickDeadband = 0;
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.roll = 26.0f / config.maxBankAngle;
    // the nose already looks into the turn as far as the sticks ask for: the craft is tilted 10 degrees forward and
    // banked 26 seen from the track
    state.headCheckOffset = bankLookIntoTurnOffset(in.roll, in.speed);
    float bodyPitch = 10.0f * DEG;
    float bodyRoll = 26.0f * DEG;
    bankRotateTilt(&bodyPitch, &bodyRoll, state.headCheckOffset);
    in.measuredPitchDeg = bodyPitch / DEG;
    in.measuredRollDeg = bodyRoll / DEG;
    // travelling straight along the track, which is to the left of the nose
    float bodyDragX;
    float bodyDragY;
    inPlaneDrag(-0.3f * cosf(state.headCheckOffset), 0.3f * sinf(state.headCheckOffset), bodyPitch, bodyRoll, &bodyDragX, &bodyDragY);
    bankDragToLevelFrame(bodyDragX, bodyDragY, bodyPitch, bodyRoll, &in.dragX, &in.dragY);   // as bankModeUpdate() does
    const float offsetBefore = state.headCheckOffset;
    bankCompute(&config, &in, &state, &out);
    EXPECT_NEAR(offsetBefore, state.headCheckOffset, 1e-5f);   // the look holds
    // the ball adds nothing: the yaw rate is only the feedforward for the bank the craft is in
    EXPECT_NEAR(0.0f, state.ballIntegrator, 1e-3f);
    float heldPitch = bodyPitch;
    float heldBank = bodyRoll;
    bankRotateTilt(&heldPitch, &heldBank, -state.headCheckOffset);
    EXPECT_NEAR(26.0f * DEG, heldBank, 1e-4f);
    const float bankTan = tanf(heldBank);
    const float speed = 0.3f - config.speedDeadband / 1000.0f;
    EXPECT_NEAR(bankTurnFeedforward(bankTan, speed, state.turnGain), out.yawRateDps, 1e-2f);
}

TEST(BankModeTest, SpeedForceBrakesAtTheBottomOfTheSpeedStick)
{
    EXPECT_FLOAT_EQ(0.0f, bankSpeedForce(0.0f, 0.0f, 0.5f, 2.0f, 4.0f));
    EXPECT_NEAR(-0.4f, bankSpeedForce(0.0f, 0.2f, 0.5f, 2.0f, 4.0f), 1e-6f);    // moving forwards: pushed back
    EXPECT_GT(bankSpeedForce(0.0f, -0.2f, 0.5f, 2.0f, 4.0f), 0.0f);             // moving backwards: pushed forward
}

TEST(BankModeTest, SpeedForceHoldsTheFeedforwardAtSteadySpeed)
{
    // at the target speed the error is 0, so the force is the one that holds that speed against the drag
    EXPECT_FLOAT_EQ(0.1f, bankSpeedForce(0.1f, 0.1f, 0.5f, 2.0f, 4.0f));
    EXPECT_FLOAT_EQ(0.4f, bankSpeedForce(0.4f, 0.4f, 0.5f, 2.0f, 4.0f));
    EXPECT_FLOAT_EQ(0.5f, bankSpeedForce(0.5f, 0.5f, 0.5f, 2.0f, 4.0f));
}

TEST(BankModeTest, OnlyTheBottomThirdOfTheSpeedStickBrakes)
{
    const float v = 1.0f;                   // flying fast
    // a quarter of the stick asks for 0.25 and brakes: pushed back
    EXPECT_NEAR(0.25f + 2.0f * (0.25f - v), bankSpeedForce(0.25f, v, 0.5f, 2.0f, 4.0f), 1e-6f);
    // 0.4 asks for 0.4 and coasts: only the force that holds 0.4, the drag slows the craft to it
    EXPECT_FLOAT_EQ(0.4f, bankSpeedForce(0.4f, v, 0.5f, 2.0f, 4.0f));
    // slower than the stick's speed, the whole lower half drives the craft up to it
    EXPECT_NEAR(0.4f + 2.0f * 0.3f, bankSpeedForce(0.4f, 0.1f, 0.5f, 2.0f, 4.0f), 1e-6f);
}

TEST(BankModeTest, SpeedForceIsContinuousAtTheCentreAndFullAtTheTop)
{
    const float maxForce = 4.0f;
    for (const float v : { 0.0f, 0.3f, 0.5f, 1.0f }) {
        // the upper half starts from the force the centre gives, whatever the speed
        EXPECT_NEAR(bankSpeedForce(0.5f, v, 0.5f, 2.0f, maxForce), bankSpeedForce(0.5001f, v, 0.5f, 2.0f, maxForce), 1e-3f) << v;
        // and ends at full forward force
        EXPECT_FLOAT_EQ(maxForce, bankSpeedForce(1.0f, v, 0.5f, 2.0f, maxForce)) << v;
        // in a straight line
        const float centre = bankSpeedForce(0.5f, v, 0.5f, 2.0f, maxForce);
        EXPECT_NEAR(0.5f * (centre + maxForce), bankSpeedForce(0.75f, v, 0.5f, 2.0f, maxForce), 1e-5f) << v;
        // and above a third the force never drops as the stick goes up
        float previous = bankSpeedForce(0.34f, v, 0.5f, 2.0f, maxForce);
        for (float stick = 0.35f; stick <= 1.0f; stick += 0.01f) {
            const float fx = bankSpeedForce(stick, v, 0.5f, 2.0f, maxForce);
            EXPECT_GE(fx, previous - 1e-6f) << v << " " << stick;
            previous = fx;
        }
    }
}

TEST(BankModeTest, MaxForwardForceIsLiftTimesTheTangentOfTheThrustLimit)
{
    // the simulator's five inch at the hover: all of its 5.7 T/W, 77 degrees of tilt
    EXPECT_NEAR(4.41f, bankMaxForwardForce(0.0f, 5.7f), 0.01f);
    // at 40 m/s the props make a third of that: 53 degrees
    EXPECT_NEAR(1.34f, bankMaxForwardForce(0.0f, 5.7f / 3.0f), 0.01f);
    // more Lift leaves less thrust to go forwards
    EXPECT_LT(bankMaxForwardForce(2.0f, 5.7f), bankMaxForwardForce(0.0f, 5.7f));
    // no thrust to spare: no forward force
    EXPECT_NEAR(0.0f, bankMaxForwardForce(0.0f, 1.0f), 1e-3f);
}

TEST(BankModeTest, ForceToTiltIsLimitedBackAndSafeAtLowLift)
{
    const float maxTilt = 30.0f * DEG;
    EXPECT_NEAR(20.0f * DEG, bankForceToTilt(tanf(20.0f * DEG), 0.0f, maxTilt), 1e-5f);
    // forward the thrust limit applies later (bankLimitTilt()); braking stops at maxTilt
    EXPECT_NEAR(atanf(15.0f), bankForceToTilt(15.0f, 0.0f, maxTilt), 1e-5f);
    EXPECT_FLOAT_EQ(-maxTilt, bankForceToTilt(-5.0f, 0.0f, maxTilt));
    const float t = bankForceToTilt(0.9f, -1.0f, maxTilt);   // free fall: no division by zero
    EXPECT_TRUE(std::isfinite(t));
    EXPECT_LT(t, 90.0f * DEG);
}

TEST(BankModeTest, ComputeSpeedStickAtTheBottomBrakesAForwardMovingCraft)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.dragX = -0.3f;                       // travelling forwards
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_LT(out.pitchDeg, 0.0f);          // nose-up to brake
    in.dragX = 0.3f;                        // travelling backwards
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_GT(out.pitchDeg, 0.0f);
}

TEST(BankModeTest, ComputeSmallDragAtZeroSpeedIsIgnored)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.dragX = -0.015f;                     // inside the speed deadband: accelerometer trim, not speed
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(0.0f, out.pitchDeg);
}

// Flies the pitch loop against a simple drag model, the attitude loop taken as perfect. Speed is in g of drag at 1 g of
// thrust; the rotor drag grows with sqrt(thrust), as the mode assumes, so in steady flight the speed times sqrt(thrust)
// equals the horizontal thrust, and it relaxes towards that at rate k.
static float flyForwardSpeed(float stick, float startSpeed, float seconds)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.speed = stick;
    const float k = 0.5f;                   // 1/s: how fast the drag catches up with the thrust
    float v = startSpeed;
    for (float t = 0.0f; t < seconds; t += in.dt) {
        in.measuredPitchDeg = out.pitchDeg;
        const float rootThrust = sqrtf(bankThrustFromCollective(out.collective, defaultThrustToWeight()));
        // the drag along the heading, levelled as bankModeUpdate() does
        in.dragX = -v * fmaxf(rootThrust, 1.0f);
        bankCompute(bankModeConfig(), &in, &state, &out);
        const float thrustX = tanf(out.pitchDeg * DEG);   // 1 g of lift
        v += k * (thrustX / fmaxf(rootThrust, 1.0f) - v) * in.dt;
    }
    return v;
}

TEST(BankModeTest, ReleasingTheStickBrakesToAStop)
{
    const float full = tanf(30.0f * DEG);
    EXPECT_NEAR(0.0f, flyForwardSpeed(0.0f, full, 10.0f), 0.03f);
}

TEST(BankModeTest, TheSpeedTargetIsNeverNegative)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    for (float stick = 0.0f; stick <= 1.0f; stick += 0.1f) {
        in.speed = stick;
        bankCompute(bankModeConfig(), &in, &state, &out);    // at rest
        EXPECT_GE(out.pitchDeg, 0.0f) << stick;
    }
}

TEST(BankModeTest, ReleasingBrakesFasterThanCoasting)
{
    const float full = tanf(30.0f * DEG);
    const float k = 0.5f;
    const float coasting = full * expf(-k * 2.0f);
    EXPECT_LT(flyForwardSpeed(0.0f, full, 2.0f), coasting * 0.5f);
}

TEST(BankModeTest, HalfStickHoldsTheMidSpeed)
{
    const float midSpeed = 0.5f;
    const auto shaped = [](float stick) { return (stick - 0.05f) / 0.95f; };     // the 5 % stick deadband
    EXPECT_NEAR(2.0f * shaped(0.25f) * midSpeed, flyForwardSpeed(0.25f, 0.0f, 20.0f), 0.02f);
    EXPECT_NEAR(2.0f * shaped(0.5f) * midSpeed, flyForwardSpeed(0.5f, 0.0f, 20.0f), 0.02f);
    // above the centre it goes on: a force half way to all the test's props can give
    EXPECT_GT(flyForwardSpeed(0.75f, 0.0f, 20.0f), 1.0f);
}

TEST(BankModeTest, AboveAThirdTheCraftCoastsDownInsteadOfBraking)
{
    // from 1.0 g of drag, the stick pulled to just above a third and to just below: the same speed asked for, near
    // enough, but only below a third does the craft brake to it; above, its drag slows it, more gently
    const float coasting = flyForwardSpeed(0.37f, 1.0f, 2.0f);
    const float braking = flyForwardSpeed(0.33f, 1.0f, 2.0f);
    EXPECT_LT(braking, coasting - 0.1f);
    // and both end at the stick's speed
    const auto shaped = [](float stick) { return (stick - 0.05f) / 0.95f; };     // the 5 % stick deadband
    EXPECT_NEAR(2.0f * shaped(0.37f) * 0.5f, flyForwardSpeed(0.37f, 1.0f, 30.0f), 0.03f);
}

TEST(BankModeTest, ForwardTiltIsLimitedByTheThrustAndBrakingByTheTiltAngle)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    // far too slow for the target: tilts past bank_max_brake_angle, up to what the thrust allows
    in.speed = 1.0f;
    for (int i = 0; i < 100; i++) {     // the forward tilt grows at BANK_FORWARD_TILT_RATE
        bankCompute(bankModeConfig(), &in, &state, &out);
    }
    EXPECT_GT(out.pitchDeg, bankModeConfig()->maxBrakeAngle + 10.0f);
    // far too fast at the bottom of the stick: tilts back no further than bank_max_brake_angle
    in.speed = 0.0f;
    in.dragX = -2.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(-(float)bankModeConfig()->maxBrakeAngle, out.pitchDeg);
}

TEST(BankModeTest, FullStickTiltsAsFarAsTheThrustAllowsBelowTheTopSpeed)
{
    // as full throttle in acro: accelerating from the hover the tilt is all the thrust allows (5.7 T/W lets the craft
    // tilt 77 degrees)
    pgResetAll();
    const float tw = defaultThrustToWeight();
    const float thrustLimit = acosf(1.0f / (1.0f + 0.75f * (tw - 1.0f))) / DEG;
    for (const float speed : { 0.0f, 0.3f }) {
        bankState_t state = {};
        bankOutputs_t out = {};
        bankInputs_t in = centredInputs();
        in.speed = 1.0f;
        for (int i = 0; i < 100; i++) {     // the forward tilt grows at BANK_FORWARD_TILT_RATE
            in.dragX = -speed;
            in.measuredPitchDeg = out.pitchDeg;
            bankCompute(bankModeConfig(), &in, &state, &out);
        }
        EXPECT_NEAR(thrustLimit, out.pitchDeg, 0.1f) << speed;
    }
    // and less where the props make a third of the thrust, close to the simulator's top speed
    bankState_t state = {};
    state.thrustTrim = 1.0f / 3.0f;
    state.forwardTilt = 1.5f;       // already tilted forward, so the slew does not hold it
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.speed = 1.0f;
    in.dragX = -0.8f;
    in.armed = true;                // disarmed, the trim is reset to 1
    in.motorsSaturated = true;      // and not measured: it only leaks
    bankCompute(bankModeConfig(), &in, &state, &out);
    const float weakLimit = acosf(1.0f / (1.0f + 0.75f * (tw * state.thrustTrim - 1.0f))) / DEG;
    EXPECT_NEAR(weakLimit, out.pitchDeg, 1.0f);
    EXPECT_LT(out.pitchDeg, thrustLimit - 10.0f);
}

// Flies forwards with the speed stick held, the attitude following the target, the configured T/W right, and the rotor
// drag in the rotor plane balancing the forward thrust (D = T * tan(theta)). Tilted, that drag also holds part of the
// weight. Only the raw accelerometer sees the drag: the filtered drag stays at 0, so the speed loop asks for the maximum
// tilt and the test is about the thrust trim alone. Returns the vertical acceleration in g.
static float cruiseWithRotorDrag(float trueTw, float seconds)
{
    pgResetAll();
    bankModeConfig_t config = *bankModeConfig();
    config.thrustToWeight = lrintf(trueTw * 100.0f);
    config.stickDeadband = 0;
    config.midSpeed = 19;           // half stick then asks for a 30 degree tilt from rest (feedforward + P)
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = true;
    in.speed = 0.5f;
    float verticalAccel = 0.0f;
    for (float t = 0.0f; t < seconds; t += in.dt) {
        const float theta = out.pitchDeg * DEG;
        const float thrust = bankThrustFromCollective(out.collective, trueTw);
        const float drag = thrust * tanf(theta);
        verticalAccel = thrust * cosf(theta) + drag * sinf(theta) - 1.0f;
        in.accX = -drag;
        in.accZ = thrust;
        in.measuredPitchDeg = out.pitchDeg;
        bankCompute(&config, &in, &state, &out);
    }
    EXPECT_GT(out.pitchDeg, 25.0f);     // really flying forwards
    return verticalAccel;
}

TEST(BankModeTest, CentreLiftStickHoldsTheHeightInForwardFlight)
{
    // the drag's lift is a third of the weight at 30 degrees; the trim takes it off, apart from what its leak leaves
    EXPECT_NEAR(0.0f, cruiseWithRotorDrag(3.0f, 30.0f), 0.03f);
}

TEST(BankModeTest, VerticalForceIsOneGInSteadyForwardFlight)
{
    // pitched forwards with the rotor drag holding part of the weight: no vertical acceleration
    EXPECT_NEAR(1.0f, bankCalcVerticalForce(-sinf(15.0f * DEG), 0.0f, cosf(15.0f * DEG), 15.0f, 0.0f), 1e-5f);
}

// Holds the lift stick at the centre after a climb or a sink has been started, with the plant's thrust
// trueTw * collective ^ 1.5. Returns the vertical speed at the end, m/s.
static float centreStickAfterVerticalSpeed(float vz0, float seconds)
{
    pgResetAll();
    bankModeConfig_t config = *bankModeConfig();
    config.stickDeadband = 0;
    const float trueTw = config.thrustToWeight / 100.0f;
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = true;
    float vz = vz0;
    for (float t = 0.0f; t < seconds; t += in.dt) {
        const float thrust = bankThrustFromCollective(out.collective, trueTw);
        in.accZ = thrust;
        bankCompute(&config, &in, &state, &out);
        vz += (thrust - 1.0f) * 9.80665f * in.dt;
    }
    return vz;
}

TEST(BankModeTest, CentreLiftStickIsOneGAndLeavesAClimbOrASinkAlone)
{
    // the stick is a force, not a vertical speed: nothing damps the climb or the sink, the pilot stops it
    EXPECT_NEAR(5.0f, centreStickAfterVerticalSpeed(5.0f, 15.0f), 0.1f);
    EXPECT_NEAR(-5.0f, centreStickAfterVerticalSpeed(-5.0f, 15.0f), 0.1f);
}

TEST(BankModeTest, LiftStickLeavingTheCentreDoesNotStepTheCollective)
{
    // moving the stick out of the deadband must not step the collective: a step there is a jolt every time the stick
    // crosses the edge
    pgResetAll();
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = true;
    in.accZ = 1.0f;
    float last = -1.0f;
    float biggestStep = 0.0f;
    for (float lift = 0.45f; lift <= 0.60f; lift += 0.0005f) {
        bankState_t state = {};
        in.lift = lift;
        bankCompute(bankModeConfig(), &in, &state, &out);
        if (last >= 0.0f) {
            biggestStep = fmaxf(biggestStep, fabsf(out.collective - last));
        }
        last = out.collective;
    }
    EXPECT_LT(biggestStep, 0.005f);
}

TEST(BankModeTest, ThrustTrimAddsThrustOnlyWhileSinkingAndLeaksBackToOne)
{
    float trim = 1.0f;
    // the craft reads 1 g but the model says more (on the ground with the lift stick up): no thrust is added
    for (int i = 0; i < 100; i++) {
        trim = bankUpdateThrustTrim(trim, 1.0f, 1.5f, true, 0.01f);
    }
    EXPECT_FLOAT_EQ(1.0f, trim);
    // 1.6 times the modelled vertical force (braking into the air): the trim follows within a few tenths of a second
    for (int i = 0; i < 30; i++) {
        trim = bankUpdateThrustTrim(trim, 1.6f, 1.0f, true, 0.01f);
    }
    EXPECT_GT(trim, 1.5f);
    EXPECT_LE(trim, 1.6f);
    // nothing to measure: it goes back to 1
    for (int i = 0; i < 500; i++) {
        trim = bankUpdateThrustTrim(trim, 0.0f, 0.0f, false, 0.01f);
    }
    EXPECT_NEAR(1.0f, trim, 0.01f);
    // a third of the modelled vertical force, sinking (fast forward flight): the trim adds the thrust back
    for (int i = 0; i < 100; i++) {
        trim = bankUpdateThrustTrim(trim, 0.33f, 1.0f, true, 0.01f);
    }
    EXPECT_NEAR(0.33f, trim, 0.03f);
    // back at 1 g it holds what it has, up to the slow leak, instead of dropping it
    const float held = trim;
    for (int i = 0; i < 100; i++) {
        trim = bankUpdateThrustTrim(trim, 1.0f, 1.0f / held, true, 0.01f);
    }
    EXPECT_NEAR(held, trim, 0.15f);
    EXPECT_LT(trim, 0.5f);
    // but goes back up at once when the craft makes more than the model says (it has slowed down)
    for (int i = 0; i < 100; i++) {
        trim = bankUpdateThrustTrim(trim, 1.5f, 1.0f, true, 0.01f);
    }
    EXPECT_GT(trim, 1.3f);
}

// Hovers at the centre of the lift stick until settled, then the props make `gain` times their thrust (the rotors
// tilted back into the air when braking). Returns the vertical acceleration, in g, `seconds` after that started.
static float centreStickAfterThrustGain(float gain, float seconds)
{
    pgResetAll();
    bankModeConfig_t config = *bankModeConfig();
    config.stickDeadband = 0;
    const float trueTw = config.thrustToWeight / 100.0f;
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = true;
    float verticalAccel = 0.0f;
    for (float t = 0.0f; t < 5.0f + seconds; t += in.dt) {
        const float thrust = bankThrustFromCollective(out.collective, trueTw) * (t < 5.0f ? 1.0f : gain);
        verticalAccel = thrust - 1.0f;
        in.accZ = thrust;
        bankCompute(&config, &in, &state, &out);
    }
    return verticalAccel;
}

TEST(BankModeTest, ThrustTrimAddsLessAtOnceWhenTheInflowFalls)
{
    // fast and tilted forward, the trim adds 2.6 times the thrust; rolled into a bank the inflow is gone
    const float trim = bankScaleTrimWithInflow(0.39f, 0.0f, 0.75f);
    EXPECT_NEAR(1.0f - 0.61f * 0.1f / 0.85f, trim, 1e-5f);
    EXPECT_FLOAT_EQ(0.39f, bankScaleTrimWithInflow(0.39f, 0.8f, 0.75f));     // more inflow: left to the trim to measure
    EXPECT_FLOAT_EQ(0.39f, bankScaleTrimWithInflow(0.39f, 0.75f, 0.75f));
    EXPECT_FLOAT_EQ(1.3f, bankScaleTrimWithInflow(1.3f, 0.0f, 0.75f));       // a trim taking thrust off is not scaled
    // a loss not from the forward inflow (a climb, the battery) is hardly touched by a small change in it
    EXPECT_NEAR(0.8f, bankScaleTrimWithInflow(0.8f, 0.0f, 0.01f), 0.02f);
}

TEST(BankModeTest, ThrustTrimTakesOffAQuickExcessOfThrust)
{
    // 0.6 g of excess at first; within half a second the trim has taken nearly all of it off
    EXPECT_LT(centreStickAfterThrustGain(1.6f, 0.5f), 0.05f);
    // a loss sinks the craft at first, and the trim adds the thrust back within a second
    EXPECT_LT(centreStickAfterThrustGain(0.8f, 0.05f), -0.1f);
    EXPECT_GT(centreStickAfterThrustGain(0.8f, 1.0f), -0.05f);
}

TEST(BankModeTest, ThrustTrimIsNotMeasuredWhileTheMixerIsSaturated)
{
    // the accelerometer reads more than the collective gives, because the mixer could not give the collective
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = true;
    in.motorsSaturated = true;
    for (int i = 0; i < 200; i++) {
        in.accZ = 1.5f;
        bankCompute(bankModeConfig(), &in, &state, &out);
    }
    EXPECT_FLOAT_EQ(1.0f, state.thrustTrim);
    in.motorsSaturated = false;
    for (int i = 0; i < 200; i++) {
        bankCompute(bankModeConfig(), &in, &state, &out);
    }
    EXPECT_GT(state.thrustTrim, 1.2f);
}

TEST(BankModeTest, AHeldHeadCheckInStraightFlightIsNoTurn)
{
    // flying straight ahead at speed, the nose held 45 degrees off the travel: the forward tilt is body roll now,
    // but there is no bank in the travel frame, so no turn is fed forward and the turn gain learns nothing
    pgResetAll();
    bankModeConfig_t config = *bankModeConfig();
    config.stickDeadband = 0;
    config.ballDeadband = 0;
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.speed = 0.6f;
    in.yaw = 0.5f;                                  // head check, held
    state.headCheckOffset = 0.5f * config.headCheckRange * DEG;
    state.turnGain = 19.0f;
    // the drag of straight travel seen from the nose 45 degrees off it: no slip in the travel frame
    const float offset = state.headCheckOffset;
    in.dragX = -0.3f * cosf(offset);
    in.dragY = 0.3f * sinf(offset);
    for (int i = 0; i < 300; i++) {
        bankCompute(&config, &in, &state, &out);
    }
    EXPECT_GT(fabsf(out.rollDeg), 5.0f);           // the forward tilt is in the body roll
    EXPECT_NEAR(0.0f, out.yawRateDps, 0.5f);
    EXPECT_FLOAT_EQ(19.0f, state.turnGain);
}

TEST(BankModeTest, ThrustTrimIsNotMeasuredWhileDisarmed)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.accZ = 1.5f;
    for (int i = 0; i < 100; i++) {
        bankCompute(bankModeConfig(), &in, &state, &out);
    }
    EXPECT_FLOAT_EQ(1.0f, state.thrustTrim);
    EXPECT_NEAR(hoverCollective(defaultThrustToWeight()), out.collective, 1e-5f);
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
    rcCommand[THROTTLE] = 1000;     // speed stick at the bottom
    testThrottleStatus = THROTTLE_LOW;
    DISABLE_ARMING_FLAG(ARMED);
    testMotorMixRange = 0.0f;
}

TEST(BankModeTest, UpdateUsesTheConfiguredThrustToWeightAndTheTrimAddsWhatIsMissing)
{
    setUpUpdate();
    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_FLIGHT_MODE(BANK_MODE);
    timeUs_t now = 1000;
    for (int i = 0; i < 2000; i++) {
        acc.accADC.z = bankThrustFromCollective(bankModeGetCollective(), 4.0f) * 512.0f;     // the real T/W is lower
        bankModeUpdate(now);
        now += 10000;
    }
    // nothing is learnt into the thrust to weight, but the craft sinks below 1 g, and the trim raises the collective
    // towards the real hover collective
    EXPECT_FLOAT_EQ(defaultThrustToWeight(), bankModeGetThrustToWeight());
    EXPECT_GT(bankModeGetCollective(), hoverCollective(defaultThrustToWeight()) + 0.03f);
    EXPECT_GT(bankThrustFromCollective(bankModeGetCollective(), 4.0f), 0.95f);
}

TEST(BankModeTest, DebugModeBankLogsTheEstimates)
{
    setUpUpdate();
    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_FLIGHT_MODE(BANK_MODE);
    debugMode = DEBUG_BANK;
    memset(debug, 0, sizeof(debug));
    acc.accADC.x = -0.3f * 512.0f;      // flying forwards
    rcCommand[THROTTLE] = 1500;         // half the speed stick
    testThrottleStatus = THROTTLE_HIGH;
    timeUs_t now = 1000;
    for (int i = 0; i < 200; i++) {
        acc.accADC.z = bankThrustFromCollective(bankModeGetCollective(), defaultThrustToWeight()) * 512.0f;
        bankModeUpdate(now);
        now += 10000;
    }
    EXPECT_GT(debug[0], 200);           // speed estimate, 0.001 g
    EXPECT_GT(debug[1], 0);             // target speed
    EXPECT_NEAR(bankModeGetThrustToWeight() * 100.0f, debug[4], 1.0f);
    debugMode = DEBUG_NONE;
}

TEST(BankModeTest, UpdateMirrorsTheModeFlagAndClearsWhenInactive)
{
    setUpUpdate();
    bankModeUpdate(1000);
    EXPECT_FALSE(bankModeIsActive());

    ENABLE_FLIGHT_MODE(BANK_MODE);
    bankModeUpdate(2000);
    EXPECT_TRUE(bankModeIsActive());
    EXPECT_NEAR(hoverCollective(defaultThrustToWeight()), bankModeGetCollective(), 1e-5f);

    testDeflection[FD_ROLL] = 1.0f;     // roll stick right = bank right
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
