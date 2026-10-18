# Plan: explore a velocity-based Lift stick for bank mode

## Context

Today the Lift stick in bank mode commands force. The owner wants to try making it command vertical velocity in the world frame, with a simple shape:

- **Centre stick:** hold zero world-z velocity (the drone keeps its height, whatever Bank and Speed are doing).
- **Above centre:** scale up toward maximum force at full stick.
- **Below centre:** scale down toward zero force at full low stick.

Only the centre behaviour has to be exact. Each side just has to reach its end point (max force or zero force).

This plan sets out the **process**. It does not cover the investigation or the design. Those happen in auto mode once the plan is approved. Baseline: branch `banked-mode-init` at `d178c2287`, with a clean tree.

## Ground rules (for the whole exploration)

- **No commits** in either repo. The owner commits.
- Firmware changes are for flight behaviour only. Diagnostics come from `DEBUG_BANK`, the existing `bankModeGet*` getters, sim plant state or a temporary unit-test harness. No new getters or debug fields just for logging.
- Code follows the Betaflight conventions in CLAUDE.md: terms (Speed/Lift/Bank vs thrust), comment style, `USE_BANK_MODE` gating, a PG version bump if the settings struct changes, and as few new settings as possible. The 4-setting rewrite is the bar.
- Leave `project_plan.md` alone until a final design is chosen.
- **Sensors: only what every drone has.** That means accelerometer, gyro / attitude estimate, motor outputs, and battery voltage/current if useful. No baro, no GPS, no rangefinder, even if the sim or a target would provide them.
- **Plausible on a real drone, not just the sim.** Each design has to cope with accelerometer noise and vibration, accelerometer bias and drift (integrating accel has no absolute reference), thrust that changes with battery sag and prop wash, and the loop rates and CPU budget of an F4. Anything that only works because the sim is clean gets flagged or dropped.
- **Works across airframes without retuning.** The same defaults should fly a whoop and a 5" acceptably, the way stock Betaflight's angle/alt modes do. So gains and limits are stated in normalised units (g, fraction of hover thrust), and anything airframe-specific (hover throttle, thrust to weight) is measured or learned in flight, not set by hand.

## Step 1: Investigate (auto mode)

- Read `project_plan.md` (spec, terms, open points) and the current Lift path in `src/main/flight/bank_mode.c` and `src/main/pg/bank_mode.{c,h}`, through to collective/thrust in the `pid.c`/`mixer.c` hooks.
- Look through existing firmware features for ideas and code to reuse, even where those features normally lean on baro/GPS. Candidates: `flight/position.c` and its altitude/vertical-speed estimate, `alt_hold_multirotor.c`, `autopilot_multirotor.c`, the IMU's earth-frame acceleration (`flight/imu.c`), throttle boost, anti-gravity, angle-mode throttle/tilt compensation, the hover-throttle settings, and launch control / crash recovery. For each, note what it measures, how it copes with accel bias and noise, and how it stays airframe-independent.
- Work out what an accelerometer-only vz estimate can deliver on a real drone: drift rate, how bias could be removed, vibration filtering, and what it costs in latency. Also check what the sim's accelerometer model includes (noise, bias, vibration), and whether the sim can add these so the prototypes aren't tested on a perfect sensor.
- Check whether the sim offers more than one airframe (a whoop-like model as well as the 5"), for testing across airframes.
- Read the newest owner flight log (`npm run bank:log`) for a baseline of how Lift is flown today.
- Check how `bank_unittest` and `bank-check.js` test Lift now, so each prototype can be measured the same way.

## Step 2: Design 3 implementations

Come up with three candidates that differ in a real way, not just in tuning. Likely axes of difference:

- the velocity estimate and how it is filtered,
- the control law at centre (for example P/PI on vz, or feedforward on gravity/bank plus a correction),
- how the stick blends from the centre velocity hold into the open-loop force at each end (shape, deadband, continuity at centre, when the hold hands over).

Write a short spec for each one: the idea, the stick-to-output mapping, new or changed settings, and the expected strengths and risks. For each one, also explain:
- why it should work on a real drone (handling of noise, bias, drift and battery sag),
- why it needs no retuning between a whoop and a 5" (what it normalises or learns),
- which existing firmware code it reuses or is modelled on.

## Step 3: Prototype each one, separately

- Build each prototype on its own from the clean baseline. When it is done, save it as a patch: `git diff > <scratchpad>/lift-velocity-{A,B,C}.patch`. Then restore the tree (`git checkout -- .`, after confirming that only the prototype's own edits are there). Re-apply a patch with `git apply` whenever it needs testing again.
- Add or adjust `bank_unittest` cases for each prototype. Cover centre stick holding vz≈0 across Bank/Speed changes, the full-up = max force and full-down = zero force end points, and stick continuity through centre.

## Step 4: Test each prototype in the sim (bank-sim-loop skill)

For each patch: `make test_bank_unittest`, then `npm run bank:cycle` in `$SIM_DIR`, then `bank-log.js --replay` of the owner's newest log. All at 1.62 g, taking off from the ground. Compare them on the same numbers:

- vz drift and height change at centre stick: hover, banked turns, Speed changes,
- response and overshoot when the stick returns to centre after a climb or a drop,
- whether each end point is reached, and any step or dead spot around centre,
- regressions in the existing bank-check thresholds,
- **realism:** the same checks with accelerometer noise, bias and vibration and with battery sag added. Do this in the sim if it supports them, otherwise in a temporary unit-test harness,
- **across airframes:** the same checks on a second, whoop-like airframe with unchanged settings, if the sim has one (or a different weight / thrust to weight on the 5"). Report whether the centre hold still holds.

## Step 5: Present

Return the tree to the clean baseline. Then report the three prototypes: the idea, the patch path, a diff summary, the test and sim numbers side by side, and pros, cons and a recommendation. I can publish this as a page with charts if that's useful.

## Step 6: Refine / implement (after discussion)

Once the owner has picked one, apply that patch, refine it as discussed, and run the full bank-sim-loop again. Update `project_plan.md` with the decision. Leave committing to the owner.
