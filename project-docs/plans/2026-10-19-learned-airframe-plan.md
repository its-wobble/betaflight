# Plan: learn the airframe in flight, no airframe settings (planned, not started)

Written 2026-10-19 for a later session. It builds on the race rewrite (branch `bank-race-rewrite`, [project_plan.md](../../project_plan.md) "Race rewrite"). Read that section and `flight/bank_mode.c` first.

## Goal

The owner wants no airframe-specific values in bank mode. Two settings describe the airframe today:

- `bank_drag_rate` (D): the exchange rate between the rotor drag the accelerometer feels and speed: drag reading = D × speed ÷ g.
- `bank_thrust_to_weight` (T/W): the thrust model, thrust = T/W × collective^1.5.

Both become values the firmware learns in flight. With D known, the speed estimate gets a real scale, so `bank_mid_speed` can be set in m/s and means the same on every quad. Afterwards the only settings left are pilot preferences: `bank_mid_speed` (m/s) and `bank_corner_rate` (deg/s).

The owner agreed to this on 2026-10-19, after the explanation in that session:
- The speed is airspeed, not ground speed, because the drag only feels air.
- Each flight starts from a default and refines it.
- The first take-off uses the default T/W.

## Part 1: learning D

**Principle.** The firmware already has two views of the speed:
- the drag view: the rotor-plane force, normalised by sqrt(thrust), in g of drag, with an unknown scale;
- the force view: the specific force integrated over time, in g·s, a true unit.

Over any period where the speed changes, the change in the drag view equals D × the integrated force. In steady flight both are zero and there is nothing to learn.

**Method: windowed least squares** (recommended over a gradient on the estimator's innovation, because it is easier to gate and to reason about).
1. Run a second integration next to the velocity estimate, `w`. It is integrated exactly like the velocity (heading frame, rotated by `bankHeadingRate`, horizontal components only), but with D = 1, and restarts from zero at the start of each window.
2. Windows are about 0.3 to 0.5 s. That is long enough to swamp the drag filter's lag and the accelerometer's noise, and short enough that integration drift and wind changes don't matter. Both sides use the same filtered accelerometer, so the lags match.
3. At the end of a window, take the change in the drag reading over the window, `Δd`, as a vector in the heading frame. Use the level-travel inversion (`bankLevelVelocity`) of the window-end and window-start readings, with each rotated into the heading frame of the end.
4. Accumulate `num += Δd · w` and `den += w · w`, both with exponential forgetting (time constant around 20 s of accepted windows). Then D = num ÷ den.
5. Accept a window only if:
   - it was armed throughout, moving faster than `BANK_GROUND_SPEED`, and the motors were not saturated;
   - tilt stayed under about 75 degrees;
   - |w| reached an excitation threshold (start around 0.3 g·s, which a punch or a brake gives in well under a second);
   - the vertical specific force stayed near Lift, so the craft wasn't climbing or sinking hard across tilted rotors.
6. Clamp D to 0.1 to 3 1/s, and limit how far one window can move it.

**Use.** The learned D replaces `config->dragRate` everywhere: the velocity integration, the turn feedforward and the corner speed.

**Start value.** A constant `BANK_DRAG_RATE_DEFAULT` (0.40, the sim's five inch), guarded with `#ifndef` so a sim test build can override it. Keep the learned value in a static that `bankModeUpdate` does not clear when BANK switches off: today `bankState` is zeroed on every mode exit, so the learned values must live outside it. They survive arm cycles until power-off.

**Optional cross-check, test side only.** In a steady turn, D = turn rate (gyro) × drag speed ÷ sideways force. Use it in the sim checker, not in the firmware.

**Caveats to state in comments and the plan:**
- It is airspeed. Gusts add noise, and steady wind cancels out because only changes are used.
- The drag is not purely linear (the airframe drag goes as speed squared), so D is the average over the speeds actually flown. That is wanted.

## Part 2: `bank_mid_speed` in m/s

- Speed in m/s = drag speed × g ÷ D. The setting becomes m/s ×10 (uint16, default 200 = 20 m/s). The speed target in g of drag is mid × D ÷ g.
- The reverse (10 % of mid) and the stop band are unchanged in shape.
- **Sim caveat.** The sim flies the five inch at 1.62 g, and its accelerometer reads in the local g (CLAUDE.md, `bf_bank.c`). The firmware's g is Earth's, 9.81, so in the sim a firmware "m/s" is 1.62 times the plant's real m/s. Correct for this in the checks (compare with plant speed ÷ 1.62), or fly the check at 1.0 g as well. On hardware it is exact.

## Part 3: learning T/W

**Principle.** T/W sets the start of the thrust model. The fast trim (measured thrust along the thrust axis over the model) and the vertical force loop already correct it in flight. What stays airframe-specific is the starting point.

**Method: two speeds.**
- **The base, `baseThrustToWeight`.** It replaces the setting and learns slowly. In calm conditions it absorbs the fast trim's long-run value: `base ← base × trim^(dt/τ)` with τ about 5 s, then trim ← trim ÷ (that step), so the product stays continuous.
- **The fast trim**, as now, on top. It handles inflow and transients and returns towards 1 relative to the base.

Calm conditions are all of:
- airborne (see below);
- tilt under 30 degrees;
- horizontal speed under about 0.2 g of drag, so little inflow;
- not saturated;
- vertical force within 0.2 g of Lift.
That is the manual "hover and read the collective" measurement in `air65-backup/reference.md`, done continuously.

**Airborne detection is the hard part.** On the ground the accelerometer reads 1 g at any low collective, which would teach a huge T/W. Use a latch `hasFlown`:
- It sets once the craft has been seen moving (faster than `BANK_GROUND_SPEED`), or the vertical velocity estimate has exceeded about +0.5 m/s while the vertical force exceeded 1 g.
- It clears on disarm.
- Base learning runs only while it is set.
- The existing "may be on the ground" rule for the fast trim stays.

**Start value.** `BANK_THRUST_TO_WEIGHT_DEFAULT` 5.7, high on purpose. `project-docs/hardware-notes.md` records that a value set too high is fully corrected by the trim, but one set too low only partly. A side effect: the base follows battery sag through a flight.

**Getter.** `bankModeGetThrustToWeight()` returns the learned base × trim. The sim's telemetry slot 5 and the log column "thrust to weight" then show it.

## Part 4: settings and plumbing

- `pg/bank_mode.h`: drop `thrustToWeight` and `dragRate`; `midSpeed` becomes m/s ×10; bump the PG version to 7.
- `cli/settings.c`: drop `bank_thrust_to_weight` and `bank_drag_rate`; give `bank_mid_speed` a new range.
- `DEBUG_BANK`: slot 1 (corner speed, derivable) becomes learned D ×1000, and slot 2 becomes base T/W ×100 × trim. Note the change in BANK-MODE.md.
- Sim:
  - `src/native/bf/bf_settings.c`: follow the PG.
  - `bf_bank.c`: telemetry unchanged in shape.
  - `scripts/bank-log.js`: column labels.

## Part 5: tests

**Unit tests** (`src/test/unit/bank_unittest.cc`). Give the test a tiny plant: a craft with a known D accelerating and braking along a line, producing consistent accelerometer inputs. Check that:
1. D converges from 0.2 and from 1.0 to the true value within a few windows of excitation;
2. nothing is learned in a hover or steady cruise, or below the excitation threshold;
3. nothing is learned when disarmed or saturated, and the clamp holds;
4. base T/W converges from a hover with a known hover collective, starting both high and low;
5. base T/W does not move on the ground (1 g measured, low collective, `hasFlown` clear);
6. learned values survive BANK off and on, and reset only at power-up.

**Sim** (the `bank-sim-loop` skill, then `npm run bank:cycle`):
1. **True D from the plant.** `bank-check.js` regresses the firmware's drag reading (`DEBUG_BANK` 0) on plant airspeed in the cruise windows. New row: learned D within ±15 % of that after the sortie (a band from this plan).
2. **Wrong start.** Build once with `-DBANK_DRAG_RATE_DEFAULT=0.2` and once with 1.0 (via `scripts/build-wasm.sh`, test builds only). The sortie should converge and the bench figures recover.
3. **The whoop.** Add a whoop sortie: `airframeById('whoop')`, its own gravity, from the ground. Its true D and T/W are far from the defaults. It should fly the sortie with no settings, and learned D and T/W should land on its own values. This lifts the known limit "the whoop is not checked".
4. **Benchmark.** `npm run bank:bench` must stay at or above the race rewrite's figures (105 / 97 / 111 / 131 %) once learned.
5. **Prove the new rows.** Break the learner (freeze D at the default) and show the new rows go red, as the sim's rule 6 requires.
6. **Real sticks.** Replay the owner's logs in the air. Today that is a scratch script (take off to 20 m, then the log's bank sticks); consider adding an `--airborne` option to `bank-log.js --replay`.

**Firmware:** `make test`, `make checks`, SITL, and `make CONFIG=BETAFPVG473_V2 OPTIONS="USE_BANK_MODE"`.

## Part 6: docs

- `project_plan.md`: a new section with the measured convergence times and the final bands. Remove the "find `bank_drag_rate` for the Air65" open point.
- `air65-backup/reference.md`: the "Measuring `bank_thrust_to_weight`" steps and the `bank_mid_speed` advice become obsolete. Replace them with "no airframe settings; the first flight learns". The bench test should watch `DEBUG_BANK` 1 and 2 converge.
- `project-docs/hardware-notes.md`: update the T/W note.
- `$SIM_DIR/BANK-MODE.md`: settings, debug fields, the 1.62 g caveat for m/s.

## Acceptance

- No airframe setting is left in bank mode.
- In the sim, the five inch and the whoop both fly the sortie from the defaults.
- Learned D lands within 15 % of the true value.
- The benchmark doesn't regress.
- All tests and builds pass.

## For the owner to decide during the work

- Save learned values to flash on disarm, or relearn every power-up? Relearning is recommended: no EEPROM writes from a flight mode, which is friendlier upstream.
- Show learned D and T/W on the OSD while they converge?
