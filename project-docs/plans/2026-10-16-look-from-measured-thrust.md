# Bank mode: look into the turn, from the measured thrust (experiment, no commit)

## Context
Commit 57913f425 ("look into the turn") made the nose look where the thrust pushes. It already did this as a head-check offset: `offsetTarget = look(theta, phi) + yawStick * range`, slewed at the head-check rate, and the tilt was rotated into the body frame. In flight it rocks on entry, swings against the turn on exit, and jitters at steep tilt (project_plan.md "The look rocks, snaps and jitters"). The entry rock comes from the look target jumping with the *commanded* bank. The tilt is rotated by that target while the nose lags 30 to 40 degrees, so the bank comes in as roll and then turns into pitch.

Experiment (chosen by the owner): strip the commit back to the car turn (nose along the track of a banked turn) and re-add the look the simple way. It is a head-check offset whose target comes from the **measured** thrust direction in the travel frame. The craft rolls in as a car first, and the nose comes round only as the thrust really tilts. On roll-out it follows the real thrust back. Do not commit; the owner reviews.

## Steps
1. **Revert the banking code to the car turn**: `git checkout 6602b150b -- src/main/flight/bank_mode.c src/main/flight/bank_mode.h src/test/unit/bank_unittest.cc`. Leave `project_plan.md` (the owner's uncommitted notes) untouched.

2. **Re-add the look helper** in `src/main/flight/bank_mode.c`, next to `bankSlewHeadCheck()`. Reuse the commit's formula and `BANK_LOOK_FADE_DEG 20.0f`, but its inputs are now the measured travel-frame tilt:
   ```c
   STATIC_UNIT_TESTED float bankLookIntoTurnOffset(float heldPitch, float heldBank)
   {
       const float azimuth = atan2f(cosf(heldPitch) * sinf(heldBank), fmaxf(sinf(heldPitch), 0.0f));
       return -azimuth * fminf(fabsf(heldBank) / DEGREES_TO_RADIANS(BANK_LOOK_FADE_DEG), 1.0f);
   }
   ```
   - The fade keeps tiny banks (and attitude noise at the hover) from swinging the nose 90 degrees.
   - Braking tilt counts as none.
   - It does not depend on where the nose points: `heldPitch` and `heldBank` are the world thrust direction seen from the track.

3. **Use it in `bankCompute()`** in the head check block (the old `offsetTarget = yawStick * range` line). The inputs are `heldPitch` and `heldBank`, already computed above via `bankRotateTilt(&heldPitch, &heldBank, -state->headCheckOffset)`:
   ```c
   const float offsetTarget = bankLookIntoTurnOffset(heldPitch, heldBank) + yawStick * DEGREES_TO_RADIANS(config->headCheckRange);
   const float offsetRate = DEGREES_TO_RADIANS(config->headCheckRate);
   state->headCheckOffset = bankSlewHeadCheck(previousOffset, offsetTarget, offsetRate, in->dt);
   const bool headCheckSlewing = fabsf(offsetTarget - previousOffset) > offsetRate * in->dt;
   ```
   - The last line is the commit's slewing fix. It is still needed: the look moves a little on every update in a turn, and the old `!=` test would stop the ball integrator and the turn gain learner for the whole turn.
   - Everything else stays as at 6602b150b: tilt, collective, ball and rotation path.

4. **Update the comments**: one "Looking into the turn" paragraph in the file header (from the measured thrust, a head check on top of the car turn). Add the `headCheckOffset` comment in `bank_mode.h`: look + head check.

5. **Unit tests** (`src/test/unit/bank_unittest.cc`):
   - Bring back the commit's `offsetSlewDps` helper and `LookIntoTurnOffsetPointsTheNoseWhereTheThrustPushes`, renamed to the held-tilt arguments.
   - New: on entry from level, the first update gives no look and a pure roll output. After the measured attitude is set to the banked tilt for a few updates, the offset goes into the turn and the output becomes pitch.
   - New: the head check adds on top of the look.
   - New: the ball integrator runs in a steady turn while the look follows.
   - Fix any 6602b150b test that feeds a banked measured attitude, so it allows for the look's yaw rate (as the commit did with `offsetSlewDps`).

## Verification
- `make test_bank_unittest`.
- Follow the `bank-sim-loop` skill: `cd $SIM_DIR && npm run bank:cycle` (builds `dist/sim-bank.wasm`, scripted sortie).
- Replay the owner's problem logs through the new firmware with `npm run bank:log -- <log> --replay` and `--rows … --replay`:
  - `testing-flight-logs/auto/webfpv-bank-20261017-002232.csv` around 8.3 s: entry rock, i.e. the roll target rising then collapsing.
  - `webfpv-bank-20261017-004932.csv`: exit yaw the wrong way.
  - Compare roll/pitch targets and `yaw - track` against the logged (committed) behaviour.
- Report the numbers to the owner; leave it uncommitted for them to fly with `npm run serve`, `?fw=bank`.
