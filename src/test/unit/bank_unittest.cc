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
    float bankLimitTilt(float fz, float *theta, float *phi, float thrustToWeight);
    float bankLiftFromStick(float stick, float thrustToWeight);
    float bankThrustFromCollective(float collective, float thrustToWeight);
    float bankCollectiveFromThrust(float thrust, float thrustToWeight);
    float bankCalcCollective(float fz, float theta, float phi, float thrustToWeight);
    float bankCalcVerticalThrust(float verticalForce, float pitchDeg, float rollDeg);
    float bankCalcVerticalForce(float accX, float accY, float accZ, float pitchDeg, float rollDeg);
    float bankUpdateVerticalSpeed(float verticalSpeed, float verticalForce, float dt);
    float bankVerticalDamping(float verticalSpeed);
    float bankLearnThrustToWeight(float estimate, float prior, float verticalThrust, float collective, float thrustDemand, float dt);
    float bankSlewHeadCheck(float offset, float target, float rate, float dt);
    void bankRotateTilt(float *theta, float *phi, float offset);
    void bankDragToTravelFrame(float dragX, float dragY, float offset, float *forward, float *lateral);
    void bankRemoveVerticalDrag(float *dragX, float *dragY, float pitch, float roll, float verticalSpeed, float turnGain);
    void bankDragToLevelFrame(float dragX, float dragY, float pitch, float roll, float *forward, float *left);
    float bankVelocityToTilt(float vTarget, float v, float kp, float fz, float maxTilt);
    float bankCalcBall(float dragX, float dragY, float offset, float deadband);
    float bankScaleBallIntegrator(float bankTan, float previousBankTan, float *integrator);
    float bankTurnFeedforward(float bankTan, float speed, float turnGain);
    float bankCoordinatedBankLimit(float speed, float turnGain, float maxYawRate);
    float bankLearnTurnGain(float turnGain, float bankTan, float speed, float dt, float *integrator);
    float bankStepBallController(float ball, float kp, float ki, float maxRate, float dt, float *integrator);
    void bankCompute(const bankModeConfig_t *config, const bankInputs_t *in, bankState_t *state, bankOutputs_t *out);
}

#include "gtest/gtest.h"

static const float DEG = M_PI / 180.0f;

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
    EXPECT_EQ(600, bankModeConfig()->thrustToWeight);
    EXPECT_EQ(90, bankModeConfig()->maxBankAngle);
    EXPECT_EQ(30, bankModeConfig()->maxTiltAngle);
    EXPECT_EQ(300, bankModeConfig()->ballKp);
    EXPECT_EQ(300, bankModeConfig()->ballKi);
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
    EXPECT_NEAR(1.0f, bankThrustFromCollective(bankCalcCollective(0.0f, 0.0f, 0.0f, tw), tw), 1e-5f);
    // forward pitch with no vertical command raises thrust by 1 / cos(theta)
    const float theta = 30.0f * DEG;
    EXPECT_NEAR(1.0f / cosf(theta), bankThrustFromCollective(bankCalcCollective(0.0f, theta, 0.0f, tw), tw), 1e-5f);
    // bank and pitch together
    const float phi = 20.0f * DEG;
    const float both = bankThrustFromCollective(bankCalcCollective(0.0f, theta, phi, tw), tw);
    EXPECT_NEAR(1.0f / (cosf(theta) * cosf(phi)), both, 1e-5f);
    // the vertical component of the thrust stays the weight: T * cos(tilt) == 1 + Fz
    const float t = bankThrustFromCollective(bankCalcCollective(0.2f, theta, phi, tw), tw);
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
    // the tilt uses its whole budget, which stops a quarter of the spare thrust short of the maximum
    const float tiltThrust = 1.1f + 0.75f * (tw - 1.1f);
    EXPECT_NEAR(tiltThrust, bankThrustFromCollective(bankCalcCollective(fz, theta, phi, tw), tw), 1e-4f);
}

TEST(BankModeTest, TiltLimiterKeepsAReserveForTheVerticalLoop)
{
    // a 5 inch at the simulator's gravity, asked for a 90 degree bank at the centre of the lift stick
    const float tw = 4.8f;
    float theta = 0.0f;
    float phi = 90.0f * DEG;
    bankLimitTilt(0.0f, &theta, &phi, tw);
    const float tiltThrust = 1.0f + 0.75f * (tw - 1.0f);
    EXPECT_NEAR(acosf(1.0f / tiltThrust), phi, 1e-4f);                  // about 75 degrees
    EXPECT_LT(phi, acosf(1.0f / tw) - 2.0f * DEG);                      // short of the 78 the thrust could hold
    const float collective = bankCalcCollective(0.0f, theta, phi, tw);
    EXPECT_LT(collective, 0.9f);                                        // room left to add lift
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
    // the tilt may use 1 + 0.75 * 0.2 = 1.15 g, so cos >= 1 / 1.15: 29.6 deg, and nothing is left for pitch
    EXPECT_NEAR(acosf(1.0f / 1.15f), phi, 1e-4f);
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

TEST(BankModeTest, BallIsCentredWithNoSlip)
{
    // travelling straight ahead: the drag points straight back, and there is nothing to correct
    EXPECT_FLOAT_EQ(0.0f, bankCalcBall(-0.2f, 0.0f, 0.0f, 0.0f));
    EXPECT_FLOAT_EQ(0.0f, bankCalcBall(0.0f, 0.0f, 0.0f, 0.0f));
}

TEST(BankModeTest, BallAsksTheNoseToFollowTheSlip)
{
    // drifting to the left: the drag points to the right (-Y), so the nose should turn left (positive)
    EXPECT_GT(bankCalcBall(-0.2f, -0.1f, 0.0f, 0.0f), 0.0f);
    // drifting to the right: nose turns right (negative)
    EXPECT_LT(bankCalcBall(-0.2f, 0.1f, 0.0f, 0.0f), 0.0f);
    // sideways from rest still turns towards the slip
    EXPECT_GT(bankCalcBall(0.0f, -0.1f, 0.0f, 0.0f), 0.0f);
}

TEST(BankModeTest, BallIsProportionalToTheLateralDragWithNoSpeedFade)
{
    EXPECT_NEAR(0.1f, bankCalcBall(-0.2f, -0.1f, 0.0f, 0.0f), 1e-6f);
    EXPECT_NEAR(0.2f, bankCalcBall(-0.2f, -0.2f, 0.0f, 0.0f), 1e-6f);
    // the forward drag does not change it
    EXPECT_NEAR(0.1f, bankCalcBall(-0.6f, -0.1f, 0.0f, 0.0f), 1e-6f);
}

TEST(BankModeTest, BallDeadbandIsSubtractive)
{
    EXPECT_FLOAT_EQ(0.0f, bankCalcBall(-0.2f, -0.02f, 0.0f, 0.03f));
    EXPECT_FLOAT_EQ(0.0f, bankCalcBall(-0.2f, 0.03f, 0.0f, 0.03f));
    EXPECT_NEAR(0.02f, bankCalcBall(-0.2f, -0.05f, 0.0f, 0.03f), 1e-6f);
    EXPECT_NEAR(-0.02f, bankCalcBall(-0.2f, 0.05f, 0.0f, 0.03f), 1e-6f);
}

TEST(BankModeTest, BallFlipsSignWhenReversingSoTheTailFollows)
{
    // moving backwards: the drag points forward (+X). Drifting to the left, the tail moves left,
    // so the nose turns right (negative)
    EXPECT_LT(bankCalcBall(0.2f, -0.1f, 0.0f, 0.02f), 0.0f);
    EXPECT_GT(bankCalcBall(0.2f, 0.1f, 0.0f, 0.02f), 0.0f);
    // it goes through zero between clearly forward and clearly backward travel, and is continuous
    float last = bankCalcBall(-0.2f, -0.1f, 0.0f, 0.02f);
    for (float x = -0.2f; x <= 0.2f; x += 0.005f) {
        const float ball = bankCalcBall(x, -0.1f, 0.0f, 0.02f);
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
    EXPECT_NEAR(0.0f, bankCalcBall(dragX, dragY, 30.0f * DEG, 0.0f), 1e-6f);
    EXPECT_LT(bankCalcBall(dragX, dragY, 0.0f, 0.0f), 0.0f);
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
        ball = bankCalcBall(dragX, dragY, 0.0f, 0.0f);
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

TEST(BankModeTest, ComputeSignsFollowTheSticks)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.roll = 1.0f;
    in.speed = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_GT(out.rollDeg, 0.0f);    // roll right
    EXPECT_GT(out.pitchDeg, 0.0f);   // nose-down, forward
    // the tilt adds collective so the vertical force is unchanged
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
    in.yaw = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_NEAR(collectiveBefore, out.collective, 1e-5f);
    // with a centred ball only the slew rate drives the nose, in the stick direction
    EXPECT_GT(out.yawRateDps, 0.0f);
    EXPECT_GT(state.headCheckOffset, 0.0f);
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
    bankInputs_t in = centredInputs();
    in.roll = -0.6f;                    // banked left, the ball asks for a left turn
    in.dragX = -0.3f;
    in.dragY = -0.1f;
    for (int i = 0; i < 100; i++) {
        bankCompute(bankModeConfig(), &in, &state, &out);
    }
    EXPECT_GT(state.ballIntegrator, 10.0f);
    in.roll = 0.0f;                     // rolled out, flying straight
    in.dragY = 0.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_FLOAT_EQ(0.0f, state.ballIntegrator);
    EXPECT_FLOAT_EQ(0.0f, out.yawRateDps);
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

TEST(BankModeTest, VerticalSpeedIsTakenOutOfTheDrag)
{
    // rolled 60 degrees right and pitched 20 forwards, climbing at 2 m/s with no horizontal speed: the rotors see the
    // climb in their plane. k is 0.034 g of drag per m/s, so turnGain = k * g in deg/s.
    const float k = 0.034f;
    const float turnGain = k * 9.80665f * 180.0f / M_PIf;
    const float pitch = 20.0f * DEG;
    const float roll = 60.0f * DEG;
    // the in-plane air speed is the climb's projection, and the drag opposes it
    float dragX = -k * 2.0f * -sinf(pitch);
    float dragY = -k * 2.0f * cosf(pitch) * sinf(roll);
    EXPECT_GT(fabsf(dragY), 0.05f);
    bankRemoveVerticalDrag(&dragX, &dragY, pitch, roll, 2.0f, turnGain);
    EXPECT_NEAR(0.0f, dragX, 1e-5f);
    EXPECT_NEAR(0.0f, dragY, 1e-5f);
    // nothing to take out when level or not climbing
    dragX = -0.3f;
    dragY = 0.1f;
    bankRemoveVerticalDrag(&dragX, &dragY, 0.0f, 0.0f, 2.0f, turnGain);
    bankRemoveVerticalDrag(&dragX, &dragY, pitch, roll, 0.0f, turnGain);
    EXPECT_FLOAT_EQ(-0.3f, dragX);
    EXPECT_FLOAT_EQ(0.1f, dragY);
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
    in.roll = 0.6f;
    in.measuredPitchDeg = 10.0f;
    in.measuredRollDeg = 26.0f;
    inPlaneDrag(-0.3f, 0.0f, 10.0f * DEG, 26.0f * DEG, &in.dragX, &in.dragY);
    bankCompute(&config, &in, &state, &out);
    // the ball adds nothing: the yaw rate is only the feedforward for the commanded bank
    EXPECT_FLOAT_EQ(0.0f, state.ballIntegrator);
    const float bankTan = tanf(0.6f * config.maxBankAngle * DEG);
    const float speed = 0.3f - config.speedDeadband / 1000.0f;
    EXPECT_NEAR(bankTurnFeedforward(bankTan, speed, state.turnGain), out.yawRateDps, 1e-3f);
}

TEST(BankModeTest, VelocityToTiltBrakesAtTheBottomOfTheSpeedStick)
{
    const float maxTilt = 30.0f * DEG;
    EXPECT_FLOAT_EQ(0.0f, bankVelocityToTilt(0.0f, 0.0f, 2.0f, 0.0f, maxTilt));
    EXPECT_LT(bankVelocityToTilt(0.0f, 0.2f, 2.0f, 0.0f, maxTilt), 0.0f);    // moving forwards: tilt back
    EXPECT_GT(bankVelocityToTilt(0.0f, -0.2f, 2.0f, 0.0f, maxTilt), 0.0f);   // moving backwards: tilt forward
    EXPECT_NEAR(-atanf(0.4f), bankVelocityToTilt(0.0f, 0.2f, 2.0f, 0.0f, maxTilt), 1e-6f);
}

TEST(BankModeTest, VelocityToTiltHoldsTheFeedforwardAtSteadySpeed)
{
    const float maxTilt = 30.0f * DEG;
    // at the target speed the error is 0, so the tilt is the one that gives that horizontal force
    EXPECT_NEAR(20.0f * DEG, bankVelocityToTilt(tanf(20.0f * DEG), tanf(20.0f * DEG), 2.0f, 0.0f, maxTilt), 1e-5f);
    EXPECT_NEAR(maxTilt, bankVelocityToTilt(tanf(maxTilt), tanf(maxTilt), 2.0f, 0.0f, maxTilt), 1e-5f);
}

TEST(BankModeTest, VelocityToTiltIsLimitedAndSafeAtLowLift)
{
    const float maxTilt = 30.0f * DEG;
    EXPECT_FLOAT_EQ(maxTilt, bankVelocityToTilt(5.0f, 0.0f, 2.0f, 0.0f, maxTilt));
    EXPECT_FLOAT_EQ(-maxTilt, bankVelocityToTilt(-5.0f, 0.0f, 2.0f, 0.0f, maxTilt));
    const float t = bankVelocityToTilt(0.3f, 0.0f, 2.0f, -1.0f, maxTilt);   // free fall: no division by zero
    EXPECT_TRUE(std::isfinite(t));
    EXPECT_LE(fabsf(t), maxTilt);
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

// Flies the pitch loop against a simple drag model: drag is k * speed, the horizontal thrust is the
// commanded tilt (the attitude loop is taken as perfect). Speed is in g of drag, so in steady flight it equals
// the horizontal thrust and it relaxes towards the thrust at rate k.
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
        in.dragX = -v * cosf(out.pitchDeg * DEG);    // the accelerometer sees the drag along the tilted body
        bankCompute(bankModeConfig(), &in, &state, &out);
        const float thrustX = tanf(out.pitchDeg * DEG);   // 1 g of lift
        v += k * (thrustX - v) * in.dt;
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

TEST(BankModeTest, FullStickSettlesAtTheMaximumTiltSpeed)
{
    EXPECT_NEAR(tanf(30.0f * DEG), flyForwardSpeed(1.0f, 0.0f, 20.0f), 0.03f);
}

TEST(BankModeTest, LearnThrustToWeightConvergesToTheMeasuredRatioAtHover)
{
    // at 1 g of demand the collective was 0.5, and the accelerometer saw 1.2 g: T/W is 1.2 / 0.5 ^ 1.5
    float estimate = 3.0f;
    for (int i = 0; i < 2000; i++) {
        estimate = bankLearnThrustToWeight(estimate, 3.0f, 1.2f, 0.5f, 1.0f, 0.01f);
    }
    EXPECT_NEAR(1.2f / powf(0.5f, 1.5f), estimate, 1e-3f);
}

TEST(BankModeTest, LearnThrustToWeightOnlyLearnsFromOneToThreeG)
{
    EXPECT_FLOAT_EQ(2.0f, bankLearnThrustToWeight(2.0f, 2.0f, 0.5f, 0.0f, 0.0f, 0.01f));     // no collective yet
    EXPECT_FLOAT_EQ(2.0f, bankLearnThrustToWeight(2.0f, 2.0f, 0.5f, 0.25f, 0.5f, 0.01f));    // lift stick low
    EXPECT_FLOAT_EQ(2.0f, bankLearnThrustToWeight(2.0f, 2.0f, 1.8f, 0.9f, 3.2f, 0.01f));     // climbing hard
    EXPECT_LT(bankLearnThrustToWeight(2.0f, 2.0f, 0.8f, 0.6f, 1.2f, 0.01f), 2.0f);          // just above 1 g is learnt
    EXPECT_GT(bankLearnThrustToWeight(2.0f, 2.0f, 1.6f, 0.7f, 1.4f, 0.01f), 2.0f);          // so is a 45 degree bank
}

TEST(BankModeTest, LearnThrustToWeightIsLimitedAroundThePrior)
{
    float high = 2.0f;
    float low = 2.0f;
    for (int i = 0; i < 2000; i++) {
        high = bankLearnThrustToWeight(high, 2.0f, 3.0f, 0.5f, 1.0f, 0.01f);     // measures 6
        low = bankLearnThrustToWeight(low, 2.0f, 0.2f, 0.5f, 1.0f, 0.01f);
    }
    EXPECT_NEAR(5.0f, high, 1e-5f);   // 2.0 * 2.5
    EXPECT_FLOAT_EQ(1.0f, low);       // 2.0 / 2.5 = 0.8, held at the 1 g floor
    // never below 1: the craft cannot be asked to hover on more than all of its thrust
    EXPECT_FLOAT_EQ(1.0f, bankLearnThrustToWeight(1.1f, 1.1f, 0.2f, 0.9f, 1.0f, 10.0f));
}

TEST(BankModeTest, VerticalThrustIsAccZWhenLevelOrInACoordinatedTurn)
{
    EXPECT_NEAR(1.2f, bankCalcVerticalThrust(bankCalcVerticalForce(0.0f, 0.0f, 1.2f, 0.0f, 0.0f), 0.0f, 0.0f), 1e-6f);
    // banked 30 degrees with the ball centred: all of the specific force is along body Z
    EXPECT_NEAR(1.0f / cosf(30.0f * DEG), bankCalcVerticalThrust(bankCalcVerticalForce(0.0f, 0.0f, 1.0f / cosf(30.0f * DEG), 0.0f, 30.0f), 0.0f, 30.0f), 1e-5f);
}

TEST(BankModeTest, VerticalThrustCountsTheRotorDragThatHoldsTheCraftUp)
{
    // Steady forward flight pitched 15 degrees, no vertical acceleration: the specific force is 1 g straight up.
    // The rotor drag lies in the rotor plane (-X), so body Z reads only cos(15) of it, less than the thrust the tilt
    // compensation asks for. The learner must see 1 / cos(15), the thrust the collective was computed for.
    const float theta = 15.0f * DEG;
    const float accX = -sinf(theta);
    const float accZ = cosf(theta);
    EXPECT_NEAR(1.0f / cosf(theta), bankCalcVerticalThrust(bankCalcVerticalForce(accX, 0.0f, accZ, 15.0f, 0.0f), 15.0f, 0.0f), 1e-5f);
    // the same on the roll axis, with the drag on body Y
    EXPECT_NEAR(1.0f / cosf(theta), bankCalcVerticalThrust(bankCalcVerticalForce(0.0f, sinf(theta), accZ, 0.0f, 15.0f), 0.0f, 15.0f), 1e-5f);
}

TEST(BankModeTest, VerticalThrustIsNotLearntWhenTiltedTooFar)
{
    EXPECT_FLOAT_EQ(0.0f, bankCalcVerticalThrust(bankCalcVerticalForce(0.0f, 0.0f, 2.0f, 80.0f, 0.0f), 80.0f, 0.0f));
    EXPECT_FLOAT_EQ(2.0f, bankLearnThrustToWeight(2.0f, 2.0f, 0.0f, 0.5f, 1.0f, 0.01f));
}

// Flies forwards with the speed stick held, the attitude following the target, the configured T/W right, and the rotor
// drag in the rotor plane balancing the forward thrust (D = T * tan(theta)). Only the raw accelerometer sees the drag:
// the filtered drag stays at 0, so the speed loop asks for the maximum tilt and the test is about the learner alone.
// Returns the vertical acceleration in g.
static float cruiseWithRotorDrag(float trueTw, float seconds)
{
    pgResetAll();
    bankModeConfig_t config = *bankModeConfig();
    config.thrustToWeight = lrintf(trueTw * 100.0f);
    config.stickDeadband = 0;
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
    EXPECT_NEAR(0.0f, cruiseWithRotorDrag(3.0f, 30.0f), 2e-3f);
}

// Hovers at the centre of the lift stick with a wrong configured T/W. The accelerometer sees the real thrust,
// trueTw * collective, or 1 g while resting on the ground. Returns the vertical acceleration at the end, in g.
static float hoverWithWrongThrustToWeight(int configuredTw, float trueTw, float lift, bool onGround, float seconds, float *finalTw)
{
    pgResetAll();
    bankModeConfig_t config = *bankModeConfig();
    config.thrustToWeight = configuredTw;
    config.stickDeadband = 0;
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = true;
    in.lift = lift;
    float verticalAccel = 0.0f;
    for (float t = 0.0f; t < seconds; t += in.dt) {
        const float thrust = bankThrustFromCollective(out.collective, trueTw);
        verticalAccel = thrust - 1.0f;
        if (onGround && verticalAccel <= 0.0f) {
            in.accZ = 1.0f;     // the ground holds the rest of the weight
            verticalAccel = 0.0f;
        } else {
            onGround = false;
            in.accZ = thrust;
        }
        bankCompute(&config, &in, &state, &out);
    }
    *finalTw = state.thrustToWeight;
    return verticalAccel;
}

TEST(BankModeTest, CentreLiftStickHoldsTheHeightWithAWrongThrustToWeight)
{
    float tw;
    // configured too high: the centre stick would sink
    EXPECT_NEAR(0.0f, hoverWithWrongThrustToWeight(200, 1.7f, 0.5f, false, 20.0f, &tw), 1e-3f);
    EXPECT_NEAR(1.7f, tw, 1e-3f);
    // configured too low: the centre stick would climb
    EXPECT_NEAR(0.0f, hoverWithWrongThrustToWeight(200, 2.3f, 0.5f, false, 20.0f, &tw), 1e-3f);
    EXPECT_NEAR(2.3f, tw, 1e-3f);
}

TEST(BankModeTest, GroundContactNeverRaisesTheEstimate)
{
    float tw;
    // configured too high, on the ground at the centre stick: not enough thrust to lift off, the estimate holds
    hoverWithWrongThrustToWeight(240, 2.0f, 0.5f, true, 10.0f, &tw);
    EXPECT_NEAR(2.4f, tw, 1e-3f);
    // a little above the centre it falls until the craft lifts off, then it learns the real value
    hoverWithWrongThrustToWeight(240, 2.0f, 0.55f, true, 30.0f, &tw);
    EXPECT_NEAR(2.0f, tw, 1e-2f);
}

TEST(BankModeTest, VerticalSpeedIntegratesTheVerticalForceAndForgetsSlowly)
{
    float vz = 0.0f;
    for (int i = 0; i < 100; i++) {
        vz = bankUpdateVerticalSpeed(vz, 1.1f, 0.01f);     // 0.1 g up for a second
    }
    EXPECT_NEAR(0.98f, vz, 0.02f);
    for (int i = 0; i < 100; i++) {
        vz = bankUpdateVerticalSpeed(vz, 1.0f, 0.01f);     // then nothing: it is kept, apart from the slow leak
    }
    EXPECT_NEAR(0.98f, vz, 0.03f);
    EXPECT_LT(vz, 0.98f);
    // pitched forwards with the rotor drag holding part of the weight: still no vertical acceleration
    EXPECT_NEAR(1.0f, bankCalcVerticalForce(-sinf(15.0f * DEG), 0.0f, cosf(15.0f * DEG), 15.0f, 0.0f), 1e-5f);
}

TEST(BankModeTest, VerticalDampingOpposesTheSpeedAndIsLimited)
{
    EXPECT_FLOAT_EQ(0.0f, bankVerticalDamping(0.0f));
    EXPECT_LT(bankVerticalDamping(1.0f), 0.0f);
    EXPECT_GT(bankVerticalDamping(-1.0f), 0.0f);
    EXPECT_FLOAT_EQ(-bankVerticalDamping(20.0f), bankVerticalDamping(-20.0f));
    EXPECT_LE(fabsf(bankVerticalDamping(100.0f)), 0.5f);
}

// Holds the lift stick at the centre after a climb or a sink has been started. The plant's thrust is
// trueTw * collective * (1 - inflow * vz): climbing, the props make less thrust, which the learner sees as a lower
// thrust-to-weight. Returns the vertical speed at the end, m/s.
static float centreStickAfterVerticalSpeed(float vz0, float inflow, float seconds)
{
    pgResetAll();
    bankModeConfig_t config = *bankModeConfig();
    config.stickDeadband = 0;
    const float trueTw = config.thrustToWeight / 100.0f;
    bankState_t state = {};
    state.verticalSpeed = vz0;          // the firmware saw the climb or the sink start
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = true;
    float vz = vz0;
    for (float t = 0.0f; t < seconds; t += in.dt) {
        const float thrust = bankThrustFromCollective(out.collective, trueTw) * (1.0f - inflow * vz);
        in.accZ = thrust;
        bankCompute(&config, &in, &state, &out);
        vz += (thrust - 1.0f) * 9.80665f * in.dt;
    }
    return vz;
}

TEST(BankModeTest, CentreLiftStickStopsAClimbOrASink)
{
    EXPECT_NEAR(0.0f, centreStickAfterVerticalSpeed(5.0f, 0.0f, 15.0f), 0.3f);
    EXPECT_NEAR(0.0f, centreStickAfterVerticalSpeed(-5.0f, 0.0f, 15.0f), 0.3f);
    // the learner takes the thrust lost to inflow as a lower thrust-to-weight, which must not keep the climb going
    EXPECT_NEAR(0.0f, centreStickAfterVerticalSpeed(5.0f, 0.03f, 15.0f), 0.3f);
    EXPECT_NEAR(0.0f, centreStickAfterVerticalSpeed(-5.0f, 0.03f, 15.0f), 0.3f);
}

TEST(BankModeTest, LiftStickLeavingTheCentreDoesNotStepTheCollective)
{
    // climbing at 3 m/s, the damping asks for less lift at the centre. Moving the stick out of the deadband must
    // fade that out, not switch it off: a step there is a jolt every time the stick crosses the edge.
    pgResetAll();
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.armed = true;
    in.accZ = 1.0f;
    float last = -1.0f;
    float biggestStep = 0.0f;
    for (float lift = 0.45f; lift <= 0.60f; lift += 0.0005f) {
        bankState_t state = {};
        state.verticalSpeed = 3.0f;
        in.lift = lift;
        bankCompute(bankModeConfig(), &in, &state, &out);
        if (last >= 0.0f) {
            biggestStep = fmaxf(biggestStep, fabsf(out.collective - last));
        }
        last = out.collective;
    }
    EXPECT_LT(biggestStep, 0.005f);
}

TEST(BankModeTest, ThrustToWeightIsNotLearntWhileTheMixerIsSaturated)
{
    // the accelerometer reads more thrust than collective * T/W, because the mixer could not give the collective
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
    EXPECT_FLOAT_EQ(defaultThrustToWeight(), state.thrustToWeight);
    in.motorsSaturated = false;
    for (int i = 0; i < 200; i++) {
        bankCompute(bankModeConfig(), &in, &state, &out);
    }
    EXPECT_GT(state.thrustToWeight, defaultThrustToWeight());
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

TEST(BankModeTest, ThrustToWeightIsNotLearntWhileDisarmed)
{
    pgResetAll();
    bankState_t state = {};
    bankOutputs_t out = {};
    bankInputs_t in = centredInputs();
    in.accZ = 1.5f;
    for (int i = 0; i < 100; i++) {
        bankCompute(bankModeConfig(), &in, &state, &out);
    }
    EXPECT_FLOAT_EQ(defaultThrustToWeight(), state.thrustToWeight);
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

TEST(BankModeTest, UpdateLearnsTheThrustToWeightWhileArmedAndForgetsItOnDisarm)
{
    setUpUpdate();
    EXPECT_FLOAT_EQ(defaultThrustToWeight(), bankModeGetThrustToWeight());
    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_FLIGHT_MODE(BANK_MODE);
    timeUs_t now = 1000;
    for (int i = 0; i < 2000; i++) {
        acc.accADC.z = bankThrustFromCollective(bankModeGetCollective(), 8.0f) * 512.0f;     // the real T/W is 8
        bankModeUpdate(now);
        now += 10000;
    }
    EXPECT_NEAR(8.0f, bankModeGetThrustToWeight(), 2e-2f);
    EXPECT_NEAR(hoverCollective(8.0f), bankModeGetCollective(), 1e-3f);

    // kept across the mode being switched off and on
    DISABLE_FLIGHT_MODE(BANK_MODE);
    bankModeUpdate(now);
    EXPECT_NEAR(8.0f, bankModeGetThrustToWeight(), 2e-2f);

    DISABLE_ARMING_FLAG(ARMED);
    bankModeUpdate(now + 10000);
    EXPECT_FLOAT_EQ(defaultThrustToWeight(), bankModeGetThrustToWeight());
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

    testDeflection[FD_YAW] = -1.0f;     // yaw stick right = bank right
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
