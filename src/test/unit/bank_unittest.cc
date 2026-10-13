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
    #include "pg/pg.h"
    #include "sensors/acceleration.h"

    extern uint8_t __config_start;
    extern uint8_t __config_end;

    acc_t acc;
    attitudeEulerAngles_t attitude;
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
    float bankLearnThrustToWeight(float estimate, float prior, float accZ, float collective, float thrustDemand, float dt);
    float bankSlewHeadCheck(float offset, float target, float rate, float dt);
    void bankRotateTilt(float *theta, float *phi, float offset);
    float bankDeadband(float value, float band);
    void bankDragToTravelFrame(float dragX, float dragY, float offset, float *forward, float *lateral);
    float bankCalcForwardSpeed(float forwardDrag, float measuredTheta, float deadband);
    float bankVelocityToTilt(float vTarget, float v, float kp, float fz, float maxTilt);
    float bankCalcBall(float dragX, float dragY, float offset, float deadband);
    float bankStepBallController(float ball, float kp, float ki, float maxRate, float dt, float *integrator);
    void bankCompute(const bankModeConfig_t *config, const bankInputs_t *in, bankState_t *state, bankOutputs_t *out);
}

#include "gtest/gtest.h"

static const float DEG = M_PI / 180.0f;

// The configured thrust-to-weight ratio, as a multiple of the weight. Hover needs a collective of 1 / T/W.
static float defaultThrustToWeight(void)
{
    return bankModeConfig()->thrustToWeight / 100.0f;
}

TEST(BankModeTest, ConfigDefaults)
{
    pgResetAll();
    EXPECT_EQ(300, bankModeConfig()->thrustToWeight);
    EXPECT_EQ(45, bankModeConfig()->maxBankAngle);
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
    EXPECT_NEAR(1.0f / defaultThrustToWeight(), out.collective, 1e-5f);
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
    // forward tilt adds collective so the vertical force is unchanged
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
    EXPECT_NEAR(1.0f / defaultThrustToWeight(), out.collective, 1e-5f);       // thrust equals weight, 1 / (T/W)

    in.lift = 1.0f;
    bankCompute(bankModeConfig(), &in, &state, &out);
    EXPECT_NEAR(1.0f, out.collective, 1e-5f);       // 2 g of lift at T/W = 2
}

TEST(BankModeTest, ComputeThrottleIsLinearInLift)
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
    EXPECT_NEAR(b.collective, 2.0f * a.collective, 1e-5f);
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
    EXPECT_NEAR(1.0f / defaultThrustToWeight(), out.collective, 1e-5f);   // the hover is not disturbed either
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
    EXPECT_NEAR(1.0f / defaultThrustToWeight(), out.collective, 1e-5f);
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

TEST(BankModeTest, DeadbandIsSubtractive)
{
    EXPECT_FLOAT_EQ(0.0f, bankDeadband(0.02f, 0.02f));
    EXPECT_FLOAT_EQ(0.0f, bankDeadband(-0.01f, 0.02f));
    EXPECT_NEAR(0.03f, bankDeadband(0.05f, 0.02f), 1e-6f);
    EXPECT_NEAR(-0.03f, bankDeadband(-0.05f, 0.02f), 1e-6f);
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

TEST(BankModeTest, ForwardSpeedSignDeadbandAndTiltCorrection)
{
    EXPECT_GT(bankCalcForwardSpeed(-0.3f, 0.0f, 0.02f), 0.0f);      // forward drag is negative: forward speed
    EXPECT_LT(bankCalcForwardSpeed(0.3f, 0.0f, 0.02f), 0.0f);       // travelling backwards
    EXPECT_NEAR(0.28f, bankCalcForwardSpeed(-0.3f, 0.0f, 0.02f), 1e-6f);
    EXPECT_FLOAT_EQ(0.0f, bankCalcForwardSpeed(-0.015f, 0.0f, 0.02f));
    EXPECT_NEAR(0.28f / cosf(30.0f * DEG), bankCalcForwardSpeed(-0.3f, 30.0f * DEG, 0.02f), 1e-5f);
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
    // T/W is really 2.4: at 1 g of demand the collective was 1 / 2.0, but the accelerometer saw 1.2 g
    float estimate = 2.0f;
    for (int i = 0; i < 2000; i++) {
        estimate = bankLearnThrustToWeight(estimate, 2.0f, 1.2f, 0.5f, 1.0f, 0.01f);
    }
    EXPECT_NEAR(2.4f, estimate, 1e-3f);
}

TEST(BankModeTest, LearnThrustToWeightOnlyLearnsNearOneG)
{
    EXPECT_FLOAT_EQ(2.0f, bankLearnThrustToWeight(2.0f, 2.0f, 0.5f, 0.0f, 0.0f, 0.01f));     // no collective yet
    EXPECT_FLOAT_EQ(2.0f, bankLearnThrustToWeight(2.0f, 2.0f, 0.5f, 0.25f, 0.5f, 0.01f));    // descending
    EXPECT_FLOAT_EQ(2.0f, bankLearnThrustToWeight(2.0f, 2.0f, 1.8f, 0.8f, 1.6f, 0.01f));     // climbing or banked hard
    EXPECT_LT(bankLearnThrustToWeight(2.0f, 2.0f, 1.1f, 0.6f, 1.2f, 0.01f), 2.0f);          // just above 1 g is learnt
}

TEST(BankModeTest, LearnThrustToWeightIsLimitedAroundThePrior)
{
    float high = 2.0f;
    float low = 2.0f;
    for (int i = 0; i < 2000; i++) {
        high = bankLearnThrustToWeight(high, 2.0f, 3.0f, 0.5f, 1.0f, 0.01f);
        low = bankLearnThrustToWeight(low, 2.0f, 0.2f, 0.5f, 1.0f, 0.01f);
    }
    EXPECT_NEAR(3.2f, high, 1e-5f);
    EXPECT_FLOAT_EQ(1.0f, low);   // 2.0 * (1 - 0.6) = 0.8, held at the 1 g floor
    // never below 1: the craft cannot be asked to hover on more than all of its thrust
    EXPECT_FLOAT_EQ(1.0f, bankLearnThrustToWeight(1.1f, 1.1f, 0.2f, 0.9f, 1.0f, 10.0f));
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
        const float thrust = trueTw * out.collective;
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
    EXPECT_NEAR(1.0f / defaultThrustToWeight(), out.collective, 1e-5f);
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
}

TEST(BankModeTest, UpdateLearnsTheThrustToWeightWhileArmedAndForgetsItOnDisarm)
{
    setUpUpdate();
    EXPECT_FLOAT_EQ(defaultThrustToWeight(), bankModeGetThrustToWeight());
    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_FLIGHT_MODE(BANK_MODE);
    timeUs_t now = 1000;
    for (int i = 0; i < 2000; i++) {
        acc.accADC.z = 2.2f * bankModeGetCollective() * 512.0f;     // the real T/W is 2.2
        bankModeUpdate(now);
        now += 10000;
    }
    EXPECT_NEAR(2.2f, bankModeGetThrustToWeight(), 1e-2f);
    EXPECT_NEAR(1.0f / 2.2f, bankModeGetCollective(), 1e-3f);

    // kept across the mode being switched off and on
    DISABLE_FLIGHT_MODE(BANK_MODE);
    bankModeUpdate(now);
    EXPECT_NEAR(2.2f, bankModeGetThrustToWeight(), 1e-2f);

    DISABLE_ARMING_FLAG(ARMED);
    bankModeUpdate(now + 10000);
    EXPECT_FLOAT_EQ(defaultThrustToWeight(), bankModeGetThrustToWeight());
}

TEST(BankModeTest, UpdateMirrorsTheModeFlagAndClearsWhenInactive)
{
    setUpUpdate();
    bankModeUpdate(1000);
    EXPECT_FALSE(bankModeIsActive());

    ENABLE_FLIGHT_MODE(BANK_MODE);
    bankModeUpdate(2000);
    EXPECT_TRUE(bankModeIsActive());
    EXPECT_NEAR(1.0f / defaultThrustToWeight(), bankModeGetCollective(), 1e-5f);

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
