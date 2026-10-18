# Bank mode: look into the turn (nose follows the thrust)

## Context

Today bank mode flies like a car on a banked track. The nose follows the direction of travel, so a Bank shows as body roll. The owner turns differently in acro. In `webfpv-bank-20261016-234544.csv` (rebuilt from the world-frame accelerometer, with the speed pinned to the logged speed):
- turns at 14–35 m/s: the camera (30° uptilt) looks a median **76° into the turn** from the track (middle half 60–106°), and the **thrust points 67° in**;
- straight runs: the camera looks 10° off the track (a check on the rebuild).

So the owner pitches into the turn rather than rolling into it, like a car drifting at about 70°. The physics is unchanged: this is the head check applied by the mode, which rotates the travel-frame tilt into the drone frame without moving the travel. The owner chose: **the nose follows the thrust's horizontal direction**, behind a hard-coded feature flag to revisit later. **No commit.** The owner reviews it in the sim.

## Change (all in `src/main/flight/bank_mode.c` unless noted)

1. **Feature flag**, near the other `#define`s:
   ```c
   // TODO(bank): feature flag for looking into the turn (2026-10-16). Decide whether it becomes a setting or the only behaviour.
   STATIC_UNIT_TESTED bool bankLookIntoTurn = true;
   ```
   It is a plain variable rather than a `#define`, so the tests can fly both laws. Add `#define BANK_LOOK_FADE_DEG 20.0f`: the bank over which the look fades in from nothing.

2. **New `STATIC_UNIT_TESTED float bankLookIntoTurnOffset(float theta, float phi)`**: theta and phi are the commanded travel-frame tilt, and the result is the nose offset in radians, positive counter-clockwise like `headCheckOffset`.
   ```c
   const float azimuth = atan2f(cosf(theta) * sinf(phi), fmaxf(sinf(theta), 0.0f));   // where the thrust pushes, from the track
   return -azimuth * fminf(fabsf(phi) / DEGREES_TO_RADIANS(BANK_LOOK_FADE_DEG), 1.0f);
   ```
   - A right bank (phi > 0) gives a negative offset: the nose turns clockwise, to the centre.
   - Braking tilt counts as no forward tilt, so a brake on a straight track still looks along it.
   - The fade keeps small corrections from swinging the nose, and makes the law continuous at θ = φ = 0.
   - Physical check: offset −90° turns (θ = 0, φ) into a pure nose-down pitch of φ. See the `QuarterTurnOffsetSwapsBankAndPitchAxes` test and `bankRotateTilt()`.

3. **In `bankCompute()`**, at the head-check block (~line 706), after `theta` (forward tilt slew) and `phi` (limits) are final:
   - `offsetTarget = yawStick * range + (bankLookIntoTurn ? bankLookIntoTurnOffset(theta, phi) : 0)`. The head check adds on top. The existing `bankSlewHeadCheck()` at `headCheckRate` (360°/s) slews it. `offsetRateDps` already feeds the slew into the yaw rate, and the existing `bankRotateTilt(&theta, &phi, offset)` turns the bank into pitch. The ball, speed, held bank and inflow all rotate by `state->headCheckOffset` already, so they need no change.
   - **`headCheckSlewing` becomes "the slew hit its rate limit this step"** (`fabsf(offsetTarget - previousOffset) > rate * dt`), not "the offset changed". With the look following theta and phi, the offset moves a little on every update in a turn. Under the old test the ball integrator and the turn-gain learner would never run in a turn.

4. **Comments and docs**:
   - `bank_mode.c` header: add a "Looking into the turn" paragraph beside "Head check", and update the stick-mapping line for Bank.
   - `bank_mode.h`: `headCheckOffset` now holds the look into the turn as well as the head check.
   - `project_plan.md`: in Terms, the Bank bullet's "the nose is turned to follow the turn". Add an Implemented entry with the acro log figures above, and a "To look at" item for the flag.

## Tests (`src/test/unit/bank_unittest.cc`)

- Declare `bankLookIntoTurnOffset` and `extern bool bankLookIntoTurn`. Add an RAII guard (`LookAlongTheTrack`) that sets the flag false and restores it, for existing tests that check the travel-frame law split into body roll/pitch. Use it only where a test fails because of the new law, and say why in a comment.
- New tests:
  - `LookIntoTurnOffset…`: 0 straight (φ = 0, any θ). ∓90° for a pure right/left bank past the fade. About −67° at θ 20°, φ 60° (the thrust azimuth). −90° for a braking θ < 0. A quarter at φ 5°. 0 at θ = φ = 0.
  - Compute, steady full right bank with the offset settled: body roll target ≈ 0, pitch target ≈ the total tilt, collective and total tilt equal to the flag-off run (the same world-frame thrust).
  - Compute: the head check adds to the look offset.
  - Compute: in a steady turn where theta moves slowly, the ball integrator still integrates (the new slewing test).

## Verification

Follow the `bank-sim-loop` skill:
1. `make test_bank_unittest` (all tests, both flag states covered).
2. `cd $SIM_DIR && npm run bank:cycle`. Expected changes, to report and not to fix by widening bands:
   - `turn_ball_g` averages the raw body Y accelerometer. With the nose in, body Y lies along the track and reads the forward drag, so it will move out of its band.
   - `turn_slip_deg` (info) will read the look offset, about −60 to −90°.
   - `turn_roll_tracking_deg` and `turn_coordination_dps` should still pass once the offset has settled. The head check rows are flown straight and should not change.
   - The sortie's rows are the owner's to change (thresholds rule).
3. `npm run bank:log -- --replay` on the owner's newest bank flight, flag on against off. Compare the pitch/roll targets, the yaw rate and the slip through the turns and hard reversals.
4. Hand over for a hand flight: `npm run serve`, `http://localhost:8000/?fw=bank`.

## Risks to watch in the sim

- **Nose lag**: the tilt is rotated by the *commanded* offset, but the nose follows at most `bank_yaw_max_rate` (360°/s, shared with the turn rate). This is the known ~10° track error of a full head check, and it now happens on every turn entry and reversal (a hard reversal is a 180° yaw in about 0.5 s). If turn entries wobble, measure first. The fixes are for later and the owner's choice: a slower look slew, or rotating by the measured nose offset.
- **Large pitch targets**: a hard bank becomes 60–85° of pitch, close to the Euler pitch singularity of the angle loop (pid.c clamps targets at 85°). The owner's acro flies there, but angle mode has not until now.
