# Bank mode: blue-sky rewrite, investigation and reimplementation plan

## Context

Bank mode flies well (owner, 2026-10-18), but it got there through months of per-symptom fixes. `flight/bank_mode.c` is ~750 lines with 16 state fields, ~33 tuned constants and 14 settings. Much of that is patches stacked on patches: the thrust trim alone has a motor-lag model, two leaks, a ground rule, inflow scaling and three gates. Speed has a reverse ramp, a tilt rule and a braking exception. Yaw has a ball PI, an integrator rescaled with tan(bank), a separate turn-gain learner and four gates. The tilt has Euler-angle composition fixes, a forward-tilt slew, a rolling freeze and a held-bank rule.

Goal: rebuild the mode from its intent, as simple as possible, accepting slightly less refined control where it buys a lot of simplicity. Behaviour does not have to match. The new code replaces the current code in the working tree. **Nothing is committed**: the owner reviews and commits.

Owner's decisions for this rewrite (2026-10-18):
- Keep the look into the turn, but only as a simple stick offset (`atan2(Bank, Speed)` scaled by a setting). Making drifting fly as well as look dot is out of scope.
- Stick laws can be simplified if the result flies similarly. Keep what they mean: Speed at the bottom brakes to a stop, Speed at the top reaches acro's top speed, Lift at the centre is 1 g, Bank is the turn.
- Settings can be reshaped freely: removed, renamed, merged, with a PG version bump and the matching sim edit. There is no target number, but fewer is better wherever a setting can be factored out.

## How simplicity is measured

These are recorded for the baseline and for the rewrite, and reported side by side:
- non-comment, non-blank lines in `bank_mode.c` (and `.h`)
- state fields in `bankState_t`
- `#define` constants (tuned numbers)
- CLI settings
- lines of hook code outside the bank files (`pid.c`, `rc.c`, `core.c`, `failsafe.c`, `mixer.c`)
- the number of distinct mechanisms: a filter, a loop, a gate, a learner or a special case each count as one

## Phase 0: freeze the baseline (nothing lost if the rewrite is worse)

1. Snapshot to `/workspaces/betaflight/.bank-snapshots/2026-10-18-pre-rewrite/`, in the same form as the existing `2026-10-17-full-bank-fixes` snapshot: a `git diff` of both repos, the built `sim-bank.wasm`, and a README with the revert commands. This folder survives a container rebuild and is already excluded from git.
2. Copy the owner's reference logs into the snapshot folder, because `testing-flight-logs/auto/` is lost on a rebuild: `webfpv-bank-20261017-232749.csv` (the latest, "handling very well"), `-175027`, `-175231` and `-151904`.
3. Add a memory entry pointing at the snapshot (the same pattern as `bank-snapshot-full-bank-fixes.md`).

## Phase 1: a scorecard that encodes the intent

Build `scorecard.mjs` in the scratchpad, not in either repo. It extends the snapshot's `hardbank.mjs` and reuses `loadSim`, `composeConfig` and `sim_bank_telemetry`. It flies at 1.62 g from the ground, and `WASM=` picks the build, so the baseline and a candidate run on the same scenarios. The output is one JSON file per build plus a diff table.

Scenarios, each mapped to a requirement:

| Requirement (intent) | Scenario | Metrics |
|---|---|---|
| Lift centre is 1 g, Speed bottom holds still | hover | vertical accel, tilt, drift |
| Speed stick sets a speed and holds it | cruise at 0.25 / 0.5 / 0.75 | settled speed, variation, vz |
| Full Speed reaches acro's top speed and holds height | full Speed for 15 s | top speed, alt change |
| Speed bottom brakes to a stop; above the bottom, the craft coasts down | brake from full, coast from full to 0.4 | time to stop, final speed, pitch peak |
| Bank is a coordinated turn | medium bank 0.6 at 0.5 | ball (travel frame), coordination, nose off track, speed held, vz |
| A held full bank stays in control | full bank at 0.15 / 0.5 / 0.75 / 1.0 | roll error, roll target flips, yaw saturation %, speed, alt loss, crash |
| Bank reversals don't nod the pitch | rev50, reversal at 1.0 | max pitch target step, pitch excursion, climb |
| Lift priority, no nose dive | Lift up / down in a bank, Lift to the bottom at 0.5 Speed | pitch target peak, alt, vz |
| Head check moves the nose, not the track | head check and return | track change, nose offset, return |
| Recovery: braking while travelling backwards, bank from the hover | backward brake, hover full bank | pitch oscillations, nose spin |
| What the owner flies | log replays: 151904 76–101 s, 175027, 175231, all of 232749 | crash, alt range, pitch swings per second, yaw saturation % |

Plus the sim's own `npm run bank:cycle` table (`tests/bank-thresholds.json`, unchanged).

**Acceptance per metric**, which is what "a little less refined" means:
- **Must**: no crash anywhere; every `bank:check` row that passes today still passes. `turn_ball_g` fails today, and that is a known artefact of the row itself.
- **Should**: within about 25 % of the baseline, or an absolute slack where the baseline is near 0 (for example ±2 degrees, ±0.5 m/s, or 5 % yaw saturation).
- A worse result that buys a large simplification is reported to the owner rather than fixed by adding a mechanism back.

Run the scorecard on the baseline wasm, save `baseline.json`, and check that it is deterministic: two runs give identical JSON.

## Phase 2: ablation, to find which lessons are real

Turn each mechanism off, one at a time, and run the scorecard. To do that, temporarily edit `bank_mode.c` in the working tree, run `npm run bank:cycle` (or the faster sync and build steps), copy `dist/sim-bank.wasm` to the scratchpad as `ablate-<id>.wasm`, then restore from the snapshot. Each result is classified as **essential** (a Must fails), **useful** (a Should degrades) or **redundant**.

Mechanisms to ablate. The owner-decided intent (Lift as a plain force, thrust priority Lift > Bank > Speed, no reverse) is not ablated.
- Thrust trim: entirely off (trim = 1); inflow scaling; the 10 Hz motor-lag model; the two leaks reduced to one; the ground rule
- Drag normalised by sqrt(thrust); trimmed against untrimmed thrust in that normalisation
- Speed sign: the reverse ramp, `bankCanReverse` (the 15 degree rule), the braking exception; speed as magnitude against along-track
- Tilt: forward-tilt slew and rolling freeze; held bank in the forward-tilt budget; `BANK_MIN_LIFT`
- Turn: coordinated bank limit; coordinated speed force; turn feedforward (ball PI only); the turn-gain learner (fixed gain); integrator rescaling with tan(bank); each integrator gate (rolling, slewing, saturated, no bank)
- Collective from the measured tilt against the commanded tilt; the floor at the thrust budget; the inverted cut
- Hooks: `pid.c` sharing the earth-referenced yaw by the measured attitude, against the targets
- Sticks: the brake-third rule; deadbands; the tilt lock on engage

The output is a ledger table, mechanism → class → evidence (the metric that moved). It goes in the scratchpad and is summarised in `project_plan.md` later.

## Phase 3: the clean-slate design

Starting point: one **force-vector formulation** in the travel frame, with Euler angles only at the output. This hypothesis is confirmed or adjusted by the Phase 2 ledger.

```
sticks    → L (Lift, g), Fy (turn force, from Bank), Fx (forward force, from Speed and v)
budget    → Tb = L + share·(T/W·trim − L);  H = sqrt(Tb² − L²)       (horizontal force available)
priority  → |Fy| ≤ min(H, turn-rate cap);  Fx ≤ sqrt(H² − Fy²);  Fx ≥ −L·tan(brake limit)
look      → rotate (Fx, Fy) by −offset (head check + look) into the nose frame
attitude  → pitch = asin(−fx/|f|), roll = atan2(fy, L)                (the angle mode's convention)
thrust    → T = L / max(cos(measured tilt), L/Tb), 0 when inverted;  collective = (T/(T/W·trim))^(2/3)
speed     → v = levelled body drag (2 Hz) / sqrt(T), along the travel frame
yaw       → ω = G·Fy_measured / v  +  Kp·ball,  with G adapted from the ball in steady turns
```

What this is expected to remove:
- `bankRotateTilt`, `bankForceToTilt` and the cos(bank) correction. Rotating a 2D force vector replaces them.
- `bankLimitTilt`, `bankMaxBank`, `bankMaxForwardForce` and `bankTiltCosMin`, replaced by one circle clip with lateral priority.
- The ball integrator, its tan(bank) rescaling and the separate learner, replaced by one adaptive gain G. The turn rate then scales with the bank and the speed by itself.
- Probably the forward-tilt slew, the rolling freeze, the coordinated speed force and the reverse logic, if the ledger says they only treated the Euler-angle bugs. Backwards or below a minimum speed, yaw is the head check only: no ball, no feedforward.
- The thrust trim cut to one estimator: measured over modelled vertical force, with the gates and the ground rule the ledger keeps.

Settings: every one is re-derived from scratch. Candidates to fold away are the ball deadband, the speed deadband, `bank_yaw_max_rate` (take it from the rate profile), `bank_max_bank_angle` (Bank as a fraction of the available turn force) and the ball Ki (gone with G). The design note lists each kept setting and why it can't be a constant or derived from something.

Write the design as a short note in the scratchpad first (equations, state, settings, and what each ledger line became), then build to it.

## Phase 4: implement

Keep the public API so the hooks and the sim glue are unchanged: `bankModeUpdate`, `bankModeIsActive`, `bankModeGetAngleTarget`, `bankModeGetYawRate`, `bankModeGetCollective`, `bankModeGetThrustToWeight`, `bankModeIsThrottleLow`.

Files:
- `src/main/flight/bank_mode.c` and `.h`: rewritten. Keep the `bankCompute(config, inputs, state, outputs)` split for testing. Comments follow CLAUDE.md's Conventions.
- `src/main/pg/bank_mode.{c,h}`: the new config struct, with a PG version bump (4 → 5).
- `src/main/cli/settings.c`: the new `bank_*` table.
- `src/test/unit/bank_unittest.cc`: rewritten, smaller, testing properties rather than constants:
  - Lift is kept at any tilt
  - the head check leaves the world-frame force unchanged
  - the priority clip order
  - the collective inverse
  - the adaptive gain converges in a steady turn
  - the stick laws' end points
  - the throttle-low and tilt-lock behaviour
- Hooks in `pid.c`, `rc.c`, `core.c`, `failsafe.c` and `mixer.c` change only if the ledger says a hook is redundant. The `pid.c` measured-attitude hook is the candidate.
- Sim (uncommitted): `src/native/bf/bf_settings.c` for the new fields, and `patches/bank/0001-bank-mode-hooks.patch` only if a hook changed (`bank-sync.sh --accept` after porting). Keep the `DEBUG_BANK` slot meanings that `bank-log.js` reads (0 speed, 2 trim, 3 forward force, 5 turn gain → G, 6 ball) so the log tooling keeps working.
- `project_plan.md`: a new section, "Rewrite (2026-10-18)", with the design, the ablation ledger summary, the new settings table and what got worse. The stick mapping table and the Terms are updated where meanings moved. The old history stays as it is, marked as describing the previous implementation.
- `$SIM_DIR/BANK-MODE.md`: only if settings that the owner's guide mentions are removed.

## Phase 5: evaluate and iterate

1. `make test_bank_unittest`, then `npm run bank:cycle` (bank-sim-loop skill). Fix until green, apart from the known `turn_ball_g`.
2. Run the scorecard against `baseline.json`. For each Must failure, find the cause in the replay rows, and add back the **smallest** mechanism that fixes it, expressed in the new formulation. Log it in the ledger. A Should regression is reported, not fixed, unless the fix is a line or two.
3. Also build and run `make test` (`check-pg-ids`), `make TARGET=SITL`, and `make TARGET=STM32F405` with the feature on, so the PG, settings and hooks compile on a real target. Run `make checks`.
4. Stop when every Must passes. Report simplicity against behaviour, side by side.

## Deliverables (all uncommitted)

- The rewritten firmware, tests and settings in `/workspaces/betaflight`, plus the matching sim edits.
- A report: the complexity table (before and after), the scorecard diff, the `bank:check` table, the ablation ledger, what got worse and why it was accepted, and the revert commands.
- A handover for the owner to fly: `npm run serve` in `$SIM_DIR`, open `/?fw=bank`, press B. Feel is not claimed as verified.

## Verification

- `make test_bank_unittest` and `make test` pass. SITL and an STM32F405 build compile. `make checks` is clean.
- `npm run bank:cycle` passes every row that passes on the baseline, with no DRIFT left unported and the `vendor/betaflight` tree clean.
- The scorecard has no crash and no Must failure across the scripted scenarios and the four owner-log replays, including the whole of `232749`.
- Determinism: two scorecard runs on the final build give identical JSON.
