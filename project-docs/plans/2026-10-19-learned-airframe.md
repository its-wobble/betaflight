# Session plan: learned airframe (2026-10-19)

The plan this session was approved on, kept as a record. What was built differs in places: the D step limit is a factor of 2 per window (not 10 %), the default's prior weight is 0.05, both learners are gated on a vertical speed from the vertical force (and T/W also on the estimate's), airborne is a climb or a sink past 0.2 m/s, and T/W learned only with Lift at 1 g or more. Afterwards the owner had the T/W learner taken out (too much code for little gain), so T/W is the fixed `BANK_THRUST_TO_WEIGHT` with the trim on top, and `DEBUG_BANK` 7 is Lift − 1 again. project_plan.md, "Learned airframe", has the result.


## Context

Implements [project-docs/plans/2026-10-19-learned-airframe-plan.md](project-docs/plans/2026-10-19-learned-airframe-plan.md) on `bank-race-rewrite` (both repos). Today two settings describe the airframe, `bank_drag_rate` (D) and `bank_thrust_to_weight` (T/W), and the Air65 has neither measured. The firmware learns both in flight from defaults, and `bank_mid_speed` becomes m/s, so the only settings left are pilot preferences (`bank_mid_speed`, `bank_corner_rate`).

Owner decisions (2026-10-19, this session):
- relearn at every power-up, with no flash writes;
- no OSD element for now;
- `bank_mid_speed` default 12 m/s (120). In the sim that keeps the five inch at today's ~20 m/s plant speed at mid stick, because the sim's firmware m/s is the plant's m/s ÷ 1.62.

Don't commit. The owner reviews first.

## Firmware: `src/main/flight/bank_mode.{c,h}`

**Learned state, kept apart from `bankState`** (which `bankModeUpdate` zeroes on every mode exit). Add `bankLearned_t` to `bank_mode.h`:
- `dragRate`: D, 1/s;
- `baseThrustToWeight`;
- the least-squares accumulators `num` and `den`;
- the window state: a running integral `W[2]`, the filtered `Wf[2]` and drag `df[2]`, snapshots of both at the window start, the window's elapsed time, and an "ok so far" flag;
- `climb` (g·s) and `hasFlown`.

It lives in a file-scope static `bankLearned` that is never cleared. Its zero state means "not started", so `bankCompute` sets the defaults when `dragRate == 0` (the same pattern as `thrustTrim == 0` today). `bankCompute` gains a `bankLearned_t *learned` argument, so the tests can drive it.

Defaults go in `bank_mode.h`, each guarded with `#ifndef` so a sim test build can override it:
- `BANK_DRAG_RATE_DEFAULT` 0.40f
- `BANK_THRUST_TO_WEIGHT_DEFAULT` 5.70f

**Part 1, learning D: windowed least squares.** This is a new `STATIC_UNIT_TESTED` helper `bankLearnDragRate(...)`, called from `bankCompute` after the velocity update. Every step:
- Integrate `W += forceForward/Left * dt`, with D = 1.
- Rotate `W`, `Wf`, `df` and both snapshots by `-headingStep`, as `velForward/Left` are. That holds everything in the current heading frame.
- `df` is the drag reading inverted with `bankLevelVelocity(dragX, dragY, roll, pitch)` and passed through a pt1 (`BANK_LEARN_FILTER_HZ`, about 5 Hz). `Wf` goes through the same pt1. The filters are linear and identical, so Δdf = D·ΔWf holds with matched lags, and a single noisy sample doesn't set the window.
- Close a window every `BANK_LEARN_WINDOW_S` (0.4 s). With `w = Wf − Wf0` and `Δd = df − df0`:
  - if the window was accepted: `num = λ·num + Δd·w` and `den = λ·den + w·w`, with λ = 1 − window/`BANK_LEARN_FORGET_S` (20 s);
  - then D = clamp(num/den, 0.1, 3), with the step per window limited to `BANK_LEARN_MAX_STEP` (about 10 %).
- The first window seeds `num/den` with a prior, `den0` = 2 windows of 0.3² g²s², and `num0 = D0·den0`, so D starts at the default and moves gradually.
- A window is accepted only if, throughout it:
  - armed, not restarted, `!motorsSaturated`;
  - moving faster than `BANK_GROUND_SPEED`;
  - `tiltCos > cos 75°`;
  - `|verticalForce − lift| < 0.3 g`, and `|velUp|` is small (a climb across tilted rotors pollutes the level inversion).
  
  Also `|w| ≥ BANK_LEARN_MIN_EXCITATION` (0.3 g·s) at the end. A rejected window restarts from the current values.
- **Use:** `learned->dragRate` replaces `config->dragRate / 100` everywhere: the velocity integration, the turn feedforward, `bankCornerSpeed`, and the up integration.

**Part 2, mid speed in m/s.**
- `midSpeed = config->midSpeed / 10 * D / GRAVITY_MSS`, in g of drag. Use the existing `GRAVITY_MSS` from `common/maths.h` if it is there; otherwise add a local 9.80665f define.
- The reverse and the stop band are unchanged.

**Part 3, learning T/W: two speeds.** This is a new helper `bankLearnThrustToWeight(...)`.
- `thrustToWeight` in `bankCompute` becomes `learned->baseThrustToWeight` everywhere: the model, the lift stick curve, the budget and the collective.
- `hasFlown`, cleared when disarmed:
  - sets when moving (≥ `BANK_GROUND_SPEED`);
  - or when `climb`, the integral of `(verticalForce − 1)` taken only while `verticalForce > 1 + BANK_AIRBORNE_MARGIN` (0.05 g, above any accelerometer bias), exceeds 0.5 m/s ÷ g.

  This is stricter than the plan's velUp test. velUp leaks at 0.2/s, so a 0.02 g bias alone would read about 1 m/s.
- Calm is all of:
  - `hasFlown` and armed;
  - not saturated, with trim measurement running (`trimModelThrust ≥ BANK_TRIM_MIN_THRUST`);
  - `tiltCos > cos 30°`;
  - horizontal speed < 0.2 g of drag;
  - `|verticalForce − lift| < 0.2 g`.
- While calm: `step = trim^(dt/BANK_LEARN_TW_S)` (5 s), then `base *= step`, `thrustTrim /= step` and `budgetTrim /= step`, so base × trim and the budget stay continuous.
- Clamp base to 1.1–10, the old setting's range.
- The trim logic (`bankUpdateThrustTrim`, the leak, inflow scaling, the may-be-on-ground rule) is unchanged.

**Getter.** `bankModeGetThrustToWeight()` returns `bankLearned.baseThrustToWeight * bankState.thrustTrim`, or the base when the trim is 0. That feeds sim telemetry slot 5.

**DEBUG_BANK.** Slot 1 changes from the corner speed to the learned D ×1000 (`[unit:0.001/s]`). Slot 2 stays the trim. Slot 7 changes from Lift − 1 to the base T/W ×100 (`[unit:0.01]`). Lift − 1 can be worked out from the stick and the base; keeping the trim separate keeps the base's input visible on hardware. This differs from the old plan, which put base × trim in slot 2.

**Comments:**
- Update the file header: "Velocity is estimated… needs no m/s calibration" changes to "D learned in flight". Add a short Limitations note: it is airspeed, and D is the average over the speeds flown.
- Fix the `thrustTrim` comment in `bank_mode.h`.

## Settings and plumbing

**No commits in either repo** (betaflight or the sim). Leave every change, including the settings and PG changes, uncommitted in the working tree for the owner to review.

- [src/main/pg/bank_mode.h](src/main/pg/bank_mode.h) and [src/main/pg/bank_mode.c](src/main/pg/bank_mode.c):
  - drop `thrustToWeight` and `dragRate`;
  - `midSpeed` becomes m/s ×10, default 120;
  - PG version 7.
- [src/main/cli/settings.c:1243-1245](src/main/cli/settings.c#L1243-L1245): drop `bank_thrust_to_weight` and `bank_drag_rate`. `bank_mid_speed` gets the range 10–600 (1–60 m/s).
- Sim (`$SIM_DIR`):
  - `src/native/bf/bf_settings.c:236-238`: follow the PG.
  - `scripts/build-wasm.sh`: append `${BANK_EXTRA_DEF:-}` to `BANK_DEF`, for test builds only.
  - `scripts/bank-log.js`: fix the summary's debug labels. It still reads `debug[4]` as T/W, which went stale in the race rewrite. Show D (slot 1), trim (2) and base T/W (7).

## Tests

**Unit tests** ([src/test/unit/bank_unittest.cc](src/test/unit/bank_unittest.cc)):
- Update the existing tests:
  - pass a `bankLearned_t` initialised to the defaults;
  - the helpers `thrustToWeight()` and `dragRate()` return the defaults;
  - fix the `bankModeConfig()` field uses.
- New tests, each on a tiny plant: a craft with a known D, level, pushed by a forward force profile (punch, then brake). It produces `accX = −D_true·v/g·sqrt(thrust)` plus the force, consistent with the heading-frame model.
  1. D converges from 0.2 and from 1.0 to the true value within a few excited windows.
  2. Nothing is learned in a hover, in steady cruise, or below the excitation threshold.
  3. Nothing is learned when disarmed or saturated, and the clamp holds.
  4. Base T/W converges in a hover with a known hover collective, starting high and starting low.
  5. Base T/W doesn't move on the ground: 1 g measured, low collective, `hasFlown` clear, and a 0.03 g bias doesn't set it.
  6. Learned values survive BANK off and on through `bankModeUpdate`.
  7. The base-to-trim transfer keeps base × trim continuous.
  8. `bank_mid_speed` in m/s maps to `mid × D / g` in g of drag.

**Sim** (the `bank-sim-loop` skill, after `make test_bank_unittest`). In `scripts/bank-check.js` and `tests/bank-thresholds.json`:
- **True D:** regress `dbg[0]` (the speed estimate, g of drag) on plant airspeed × g_local⁻¹ over the cruise windows. Add a row `learned_drag_rate_error`: |dbg[1]/1000 − D_true| / D_true, band [0, 0.15], from the plan.
- **Whoop sortie:** a second `sortie({ airframe: 'whoop65' })`. It calls `sim_set_airframe(1)` before init, as `flightcheck.js` does, with gravity from `airframeById('whoop65').gravityBase`, a 1S cell voltage and its own tune if `flightcheck.js` uses one. Its rows carry a `whoop_` prefix. The core rows (hover, cruise, brake, D error) are banded and the rest are info. This lifts "the whoop is not checked".
- **Wrong start:** use `BANK_EXTRA_DEF="-DBANK_DRAG_RATE_DEFAULT=0.2f"`, then `1.0f`, and a low and a high T/W default, each a build and `node scripts/bank-check.js`. These runs are by hand in this session, not part of `bank:cycle`.
- **Prove the new rows:** build with learning frozen (a temporary local edit that skips the update) and show `learned_drag_rate_error` and the whoop rows go red.
- `npm run bank:bench` must stay at or above 105 / 97 / 111 / 131 %.
- **Real sticks:** `npm run bank:log -- --replay` on the newest owner log, plus an airborne replay (scratch script: climb to 20 m, then the log's bank sticks).

**Firmware:**
- `make test_bank_unittest`, `make test`, `make checks`;
- `make TARGET=SITL`;
- `make CONFIG=BETAFPVG473_V2 OPTIONS="USE_BANK_MODE"`.

## Docs

- [project_plan.md](project_plan.md):
  - add a "Learned airframe (2026-10-19)" section with the measured convergence times, the bands and the DEBUG slot changes;
  - update the Terms entries for T/W and g of drag, and the race rewrite's settings list;
  - remove the "find `bank_drag_rate` for the Air65" open point.
- [air65-backup/reference.md](air65-backup/reference.md):
  - remove the "Measuring `bank_thrust_to_weight`" section, the `set bank_mid_speed = 10` advice and the `bank_thrust_to_weight` line, replacing them with "no airframe settings; the first flight learns";
  - suggest `bank_mid_speed` around 50 (5 m/s) for first indoor flights;
  - update the DEBUG_BANK table, and say the bench test watches slots 1 and 7.
- [project-docs/hardware-notes.md](project-docs/hardware-notes.md): rewrite the T/W note around the default starting high.
- `$SIM_DIR/BANK-MODE.md`: the settings, the debug fields, and the m/s caveat at 1.62 g.
- Mark the old plan file "implemented", and copy this plan to `project-docs/plans/2026-10-19-learned-airframe.md`.

## Acceptance

- No airframe setting is left.
- In the sim, the five inch and the whoop both fly the sortie from the defaults.
- Learned D is within 15 % of the true value.
- The benchmark doesn't regress.
- All tests and builds pass.
