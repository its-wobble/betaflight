# Bank mode: drop the thrust trim, keep the fixed T/W

## Context
The owner wants to try flying BANK with no thrust trim, using only the fixed thrust-to-weight setting for the five inch. The trim currently matches the props' thrust to the Lift asked for, so it also takes out the props' own vertical damping: a prop makes less thrust while climbing and more while sinking. With the trim gone, a climb or sink at centre Lift should die out by itself through prop inflow and the world-z body drag.

**T/W value (owner's choice: keep 5.70).** The owner's hover logs from 2026-10-18 (170427, 171544: level, below 1 m/s, trim at 0.99) need collective 0.3145–0.318 on a full pack (25.05 V), which is 5.66–5.70 with the collective^1.5 curve. The default 570 already holds a hover, so it stays. The plant's real full-throttle figure is 5.20 (58.7 N against 0.71 kg × 1.62 g). That difference was offered and not taken.

## Changes (betaflight)

**[src/main/flight/bank_mode.c](src/main/flight/bank_mode.c)**
- Remove the trim: `BANK_TRIM_*` constants, `bankUpdateThrustTrim`, `bankLeakThrustTrim`, `bankScaleTrimWithInflow`, the trim block in `bankCompute` (lines 296–298 and 307–316), the inflow scaling (lines 341–343) and the `inflow` local.
- Use the plain `thrustToWeight` wherever `thrustToWeight * state->thrustTrim` / `trimmedThrustToWeight` was used: the drag normalisation thrust (line 319), the lift clamp, `bankTiltThrust` and `bankCollectiveFromThrust` (lines 369–383). Drop the "the limits use the trimmed thrust" comment.
- `bankModeUpdate`: drop `.accZ` and `.motorsSaturated` (`getMotorMixRange`). Drop `#include "flight/mixer.h"` if nothing else uses it.
- `DEBUG_BANK` 2: remove the `DEBUG_SET`. The slot stays empty so the other indices don't move for the sim's log tools.
- Header comment: remove "A fast thrust trim corrects…" and change the limitation "Lift is a force: nothing holds the height or the vertical speed" to say the props' inflow and the body drag damp vertical speed at centre Lift, and that fast forward flight and battery sag leave the modelled thrust high.

**[src/main/flight/bank_mode.h](src/main/flight/bank_mode.h)**: remove the `accZ` and `motorsSaturated` inputs and the `thrustTrim`, `trimModelVertical` and `trimInflow` state fields.

**[src/test/unit/bank_unittest.cc](src/test/unit/bank_unittest.cc)**: remove the three trim prototypes (lines 79–81) and the tests `ThrustTrimTakesOffExcess…`, `OnlyATrimThatTakesThrustOffLeaksBack`, `ThrustTrimAddsLessAtOnce…` and `ThrustTrimIsNotMeasuredWhileDisarmedOrSaturated`. Drop `in.accZ` (line 105) and fix the "no trim" comment at line 381. Other tests that relied on the trim staying at 1 should now hold as they are; fix any that fail for that reason.

**PG / CLI**: no change. The `bank_thrust_to_weight` default stays 570 and the struct is unchanged, so there's no version bump.

**[project_plan.md](project_plan.md)**: add a dated section, "No thrust trim (2026-10-18)": the decision, why 5.70, and the before/after numbers below. Update the Terms entry "Thrust trim" and the "Lift stays a force" note, which says the trim cancels the props' damping.

**[air65-backup/reference.md](air65-backup/reference.md)**: with no trim, a T/W set too high no longer gets corrected, so the craft sinks at centre Lift or won't lift off. This reverses the memory note "keep 5.70 high for both crafts". The Air65 needs its measured value (the existing "Measuring" section) before it flies BANK, and the `debug[2]` trim rows in "set too high/low" and the log table get updated.

## Changes (sim, `$SIM_DIR`)
- `scripts/bank-log.js`: drop the `trim` column from the summary header and rows (debug[2] is now always 0).
- `BANK-MODE.md`: change the debug[2] description to "unused" and remove the trim from the summary text.
- `scripts/bank-check.js` (line ~96 comment) and `tests/bank-thresholds.json`: leave the thresholds alone. Report any rows that move or fail, and don't retune them.
- Follow the sim's `bank-sim-check` skill for its own docs (the PROGRESS.md entry).

## Verification (bank-sim-loop skill)
1. Before editing, run `npm run bank:cycle` on HEAD for the baseline. HEAD (2a8a2a7e6) is the revert point, and the tree is clean apart from the untracked `bank_takeoff_calibration.md`.
2. `make test_bank_unittest`, then `make test` for the core/failsafe hooks, then `npm run bank:cycle`.
3. Scratch harness in the scratchpad (five inch at 1.62 g, from the ground, full pack, run on HEAD and on the new build):
   - **Damping**: from a hover, Lift pulsed to 0.75 for 1 s then centred, and the same downward. Measure the vertical speed and how long it takes to fall below 0.3 m/s. This is the change's goal.
   - **Take-off** at centre Lift: the climb rate and whether it dies out.
   - **Full Speed at centre Lift**: height and vertical speed after 3, 10 and 20 s. Without the trim the props lose about 2/3 of their thrust at 40 m/s, so expect a sink. Also check the speed estimate against the true speed: the drag normalisation now uses the untrimmed thrust.
   - **Brake** from cruise and a **hard bank entered at speed**: the balloon and climb that the fast trim used to remove.
   - The whoop taking off from the ground in BANK at its own value of about 4.8 (memory: check whoop take-offs).
4. Replay the owner's newest log (`npm run bank:log -- --replay`), looking only for gross failures.
5. Report to the owner with numbers: what improved (damping) and what got worse (fast-flight sink, braking balloons, battery sag). No commit.
6. Update the memory `whoop-launch-in-bank.md`, since its "keep T/W high because the trim corrects it" reasoning no longer applies.
