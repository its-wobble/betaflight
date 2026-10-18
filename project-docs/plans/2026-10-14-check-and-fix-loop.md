# Bank mode: check-and-fix loop (no commits)

## Context
The user asked for a loop that checks how bank mode currently behaves in the WebFPVSimulator, then finds and fixes bugs. **Nothing is committed in either repo.** All changes stay in the working trees for the user to review.

Starting point is the last sim report (`$SIM_DIR/build/bank/report.json`, 2026-10-14): 13/15 banded rows pass.
- FAIL `cruise_vertical_accel` = +0.47 m/s² (band ±0.3), FAIL `turn_vertical_accel` = +0.300 (on the edge). The learnt T/W falls from ~3.84 at hover to 2.6 in forward flight, so the craft climbs at centre lift stick.
- Info `turn_slip_deg` = 28°: in a steady turn the nose is 28° off the track even though the ball reads centred (0.026 g). The spec says ball centred means a perfect bank, so this looks wrong.

## Loop (per the `bank-sim-loop` / `bank-sim-check` skills), up to ~5 iterations
1. `cd /workspaces/betaflight && make test_bank_unittest`
2. `cd $SIM_DIR && npm run bank:cycle`, then record the table.
3. For failing or suspicious rows, run `npm run check:bank -- --explore` and read the time series (tw, collective, acc, pitch, roll, yaw vs track).
4. Fix in the firmware (`src/main/flight/bank_mode.c` + `src/test/unit/bank_unittest.cc`), using only FC-available inputs. Then repeat from step 1.
5. Stop when every row passes, or when a remaining failure is understood and argued (e.g. the band contradicts physics). Do not edit `tests/bank-thresholds.json` and do not change sim physics.

## Planned fix A: thrust-to-weight learner (cruise/turn vertical accel)
Root-cause hypothesis: `bankLearnThrustToWeight()` (`bank_mode.c:217`) takes `accZ / collective`. In the sim, as on a real quad, the plant's per-axis body drag (`plant.c` "Airframe drag") has a body-Z component whenever the craft is tilted and moving. accZ then under-reads the thrust: in steady flight the specific force is 1 g earth-up, so accZ ≈ cosθ·cosφ, not 1/(cosθ·cosφ). The learnt T/W is too low, the collective too high, and the craft climbs.
- Fix: learn from the **earth-vertical specific force** instead. It is f_up = u·acc, where u = (−sinθ, cosθ·sinφ, cosθ·cosφ) is the body-frame vertical from the measured attitude (the same u used in `bankRotateTilt`). Learn `Tmax ≈ f_up / (collective · u_z)`. With zero vertical acceleration (f_up = 1) this is exactly the T/W that holds zero vertical accel at the current collective and tilt, which is what the centre of the lift stick promises. Horizontal drag no longer leaks in.
- Plumbing: add `accX/accY` (raw, unfiltered, like `accZ`) to `bankInputs_t` (`bank_mode.h`), fill them in `bankModeUpdate`, and pass the measured pitch/roll to the learner. Keep the existing learn window, filter, ±60% clamp and reset-on-disarm.
- Update the header comment in `bank_mode.c` (lines ~87–94, 135–137). Unit tests: adapt the existing learner tests. Add one test for a tilted, steady case with drag projected onto body Z, where the old formula under-estimates and the new one does not.
- Spec note: `project_plan.md` says the learner uses "the body-Z accelerometer". I will **not** edit the spec, but will flag the wording for the user.
- Check: the cruise/turn vertical accel rows go in band and hover is unchanged. Also check, by hand, the learnt tw vs the hover value.

## Investigation B: 28° slip with the ball centred
I will explore before deciding on a fix. Candidates:
- The ball is computed from body-frame drag rotated only by the head-check yaw offset (`bankCalcBall`, `bankDragToTravelFrame`). It ignores pitch and roll, so forward drag leaks into body Y when banked and pitched (term ∝ sinφ·sinθ).
- Or the checker's yaw/track convention is off, and the 28° is not real.

If the firmware is at fault, fix it with measured attitude only: project the drag into the level travel frame. Add a unit test and confirm the slip drops without regressing `turn_ball_g`/`turn_coordination_dps`. If the checker is at fault, report it and don't change the sim without asking.

## Also check while in there (from reading the code)
- The speed estimate uses the previous head-check offset, while the ball uses the updated one (`bankCompute`). This is minor; only align it if it matters.
- Any further anomalies the explore trace shows (brake time, head-check track change −7.9°, head return 4.8° near the band edge).

## Sim side
I expect no sim changes. If `bank:sync` reports DRIFT or the build breaks, I will port per the sim skill. That stays uncommitted as well, contrary to the skill's "commit and push" step, because the user asked for no commits. `git diff --stat vendor/betaflight` must be empty at the end.

## Verification / report
- `make test_bank_unittest` passes; `npm run bank:cycle` table before vs after, every row with its value.
- `git -C /workspaces/betaflight diff` and `git -C $SIM_DIR status` for the user's review. No commits.
- State the known limits: true attitude, no vibration or bias, and on hardware the imu attitude is biased under drag, which now feeds the learner too. Close with "harness result as above, feel awaiting the owner's flight".
