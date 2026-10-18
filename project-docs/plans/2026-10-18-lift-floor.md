# Zero Lift locks up: a fixed Lift floor (option D)

## Context

project_plan.md "To look at" → "Zero Lift locks up" (owner, 2026-10-18): with Lift at the bottom the craft can't be steered. At collective 0 the attitude still follows Speed and Bank, but nothing moves the craft, and the turn feedforward and ball are off (`lift > 0` gates).

In the owner's five flights of 2026-10-18 there are 4 to 21 zero-Lift stretches per flight, each 0.1 to 1.9 s long. Most are held at high Speed with the pitch target 50 to 83° nose down, some in a full bank. About a third reach the ground within 1.5 s, because Lift comes back with the craft steeply pitched.

Options A to D (steering floor, ease to level, tilt on edge, fixed floor) were discussed. **The owner chose D, a fixed floor at the bottom of the Lift stick, to see how it feels.** They are fine with Speed as well as Bank getting thrust at the bottom, so full Speed at the bottom becomes a powered dive. They want the floor alone first, to see whether it catches everything on its own. So there is no ease-to-level and no feedforward change in this round.

## Change: `src/main/flight/bank_mode.c` (no settings)

1. **New constant** `BANK_LIFT_FLOOR 0.2f` ("g, Lift at the bottom of the stick: enough thrust to steer"). 0.2 g is the starting value; the sim pass below compares 0.1, 0.2 and 0.3 for the owner.
2. **`bankLiftFromStick()`**:
   - Lower half: `f + 2s(1 − f)`, which runs from the floor at the bottom to 1 g at the centre.
   - Upper half: `1 + (1 − f)x + (T/W − 2 + f)x³`, which keeps the top at T/W and both halves at the same slope at the centre (the "centre isn't touchy" property).
   - The `tw <= 2` branch stays linear.
   - Update the comment and the file-header line "Lift: 0 g at the bottom".
3. **Throttle low keeps its meaning.** With a floor the collective is no longer 0 at the bottom of the stick. `bankModeIsThrottleLow()` (collective ≤ 0) feeds the `core.c` I-term reset before airmode latches, `MOTOR_STOP`, auto-disarm, runaway takeoff and failsafe's landed timer (`core.c:811,857,1002,1023,1403`, `failsafe.c:334`). Silently dropping all of those would be a hardware-safety change.
   - Add `bool throttleLow` to `bankOutputs_t` (`bank_mode.h`), set in `bankCompute()` as `liftStick <= 0.0f || collective <= 0.0f`: the stick at the bottom, or inverted.
   - `bankModeIsThrottleLow()` returns it, with its comment updated.
   - Today the two are the same condition, so nothing changes for those hooks.
4. Nothing else changes, and this is deliberate:
   - The `lift > 0` gates now pass at the bottom, so the turn feedforward and the ball run there. That is the steering.
     **Changed while doing it:** in the sim, banked drops spun the nose 150 to 180° off the track (the sink at steep bank reads as slip to the ball, and the feedforward assumes 1 g). Scaling the feedforward by Lift didn't help. So the yaw gates moved to the Lift stick (`liftStick > 0`), which is the baseline's own condition, and with that the floor changes only the thrust. See project_plan.md, "Zero Lift locks up".
   - The angles still use the `BANK_MIN_LIFT` 0.5 g geometry.
   - The trim doesn't measure below 0.3 g modelled vertical, as today.

**Expected side effects to report:**
- Armed in BANK on the ground with Lift at the bottom, the motors run at about 11 % collective, not idle. The hardware procedure arms in ANGLE.
- The fastest descent is 0.8 g, not free fall.
- The turn feedforward assumes 1 g of Lift, so at the floor a full Bank over-yaws by up to 5×. This is measured, not fixed. If it shows, offer the feedforward × Lift fix as a follow-up.

## Tests: `src/test/unit/bank_unittest.cc`

- `LiftIsZeroAtTheBottom…` becomes floor at the bottom, 1 g at the centre, T/W at the top, with equal slopes either side of the centre, and still 1.5 at the top for T/W 1.5.
- `ThrottleIsLowOnlyWithNoCollective` becomes throttle low with the Lift stick at the bottom (collective > 0 there), and not at the centre.
- New: Lift at the bottom with full Bank gives collective > 0 and a turn feedforward (yaw rate ≠ 0) at speed.
- Fix any other test that assumed collective 0 at the bottom of the stick.

## Docs

- project_plan.md: the Lift row of the stick table, the "Lift" term (bottom = the floor), and the "Zero Lift locks up" note rewritten to say what was decided (D, owner 2026-10-18), with the sim results.
- `$SIM_DIR/BANK-MODE.md`, Pitch row: "Bottom is no lift" becomes "Bottom is a little lift (0.2 g), enough to steer while dropping".
- Copy this plan to `project-docs/plans/2026-10-18-lift-floor.md`.
- No commit (the owner reviews first).

## Verification (bank-sim-loop skill)

1. `make test_bank_unittest`, then `npm run bank:cycle` in `$SIM_DIR`. `bank:check` should stay 22 of 23.
2. Scratch scenarios at 1.62 g from the ground, floor 0.1, 0.2 and 0.3, against the current build:
   - drop from 20 m at full Speed, Lift to the bottom for 1.5 s then centre: height lost, minimum vz, pitch at Lift return;
   - drop in a full bank: turn rate, slip, yaw-limit share (the feedforward over-yaw check);
   - a brake with Lift at the bottom.
3. `npm run bank:log -- --replay` over 223018, 211048 and 205131: ground contacts after zero-Lift stretches, before and after.
4. Build SITL and `CONFIG=BETAFPVG473_V2 OPTIONS="USE_BANK_MODE"` so the `core.c` and `failsafe.c` hooks still compile. Run `make test` for the core and failsafe unit tests.
5. Report the numbers and the side effects to the owner, so they can fly `?fw=bank` and decide.
