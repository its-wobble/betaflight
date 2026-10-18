# Bank mode snapshots

Revert points and parked experiments, newest first. Each one is a folder under `.bank-snapshots/`, which git ignores through `.git/info/exclude` (not `.gitignore`). It sits in `/workspaces`, so it survives a container rebuild. Each folder's README has the commands to revert to it or apply it. Drop `.bank-snapshots/` before any upstream PR.

"Go back" or "revert" from the owner means the README commands of the snapshot they name. Nothing in the snapshots is committed, because the owner reviews before committing.

## 2026-10-18: tuning round 2 harness (tools, not a revert point)

Folder: `.bank-snapshots/2026-10-18-tuning-round-2/`. Write-up: project_plan.md, "Tuning round 2 (2026-10-18)".

- **Revert:** the round started clean at 49e73edd2, so `git checkout` of the four changed files goes back; nothing in the folder is needed for that.
- **In the folder:** the rewrite's scorecard brought up to date (swapped sticks, level variants, brake-in-bank climbing and sinking, whoop65, look turn 0/100, whole-flight ensembles), the base and final `sim-bank.wasm`, four owner logs of 2026-10-18, and the loop log.
- **Why keep it:** start the next tuning pass from this scorecard rather than rebuilding one. It reads the older logs from the pre-rewrite snapshot's `logs/`.

## 2026-10-18: no thrust trim + fixed inflow thrust curve (parked)

Folder: `.bank-snapshots/2026-10-18-inflow-curve/`. Write-up: project_plan.md, "No thrust trim (2026-10-18, tried and parked)".

- **The trial:** the owner removed the measuring thrust trim, so that prop inflow and body drag damp vertical speed at centre Lift. A fixed curve then took back the thrust lost in fast flight: `share = clamp(1 − 0.8 × speed × sin(forward tilt), 0.3, 1)`.
- **Parked:** as "a layer of complexity I'm not quite ready for". The code went back to 2a8a2a7e6.
- **In the folder:** the patch, the three flown wasm builds (old trim, no trim, fixed curve), the scratch harness and the results.
- **Agreed:** the curve is fixed per drone, not learnt in flight. A learner on the force error integrates the vertical acceleration, which makes it a velocity hold with the accelerometer's bias.
- **Open when it comes back:**
  - the full-Speed bob, from the speed estimate's pitch dependence above 45 degrees
  - the braking balloon
  - a per-drone slope
  - battery sag

## 2026-10-18: Lift as a vertical speed, three prototypes (A/B/C)

Folder: `.bank-snapshots/2026-10-18-lift-velocity/`. The patches apply to d178c2287, and the tree was restored there afterwards. Nothing was committed.

The owner asked for three prototypes of a Lift stick whose centre holds zero world vertical speed (above the centre up to the maximum force, below it down to zero force), presented for discussion before choosing one:

- **A, centre brake:** P on a leaky accelerometer vertical speed, fading out to the stick ends.
- **B, velocity stick:** the inner 60 % is a climb rate target, and the outer part ramps to the force ends.
- **C:** A, plus the props' thrust-loss reading as a bias-free vertical speed anchor near the hover, calibrated after lift-off.

Findings that drive the choice:
- **Accelerometer offset:** no loop built only on the accelerometer can fix an offset in flight. The thrust trim and the vertical speed estimate share it, so the craft accelerates at minus the offset. Only the props' inflow is independent of it (thrust ratio ≈ 1 − c·vz, with c ≈ 0.72 per g·s on the five inch and 0.88 on the whoop in the sim).
- **Lasting estimate errors:** once the loop holds the estimate at its target, the leak does nothing. Any estimate error then becomes a lasting climb or sink, for example an impact that clips at 16 g, or the sim's accelerometer filter starting at 0. All three designs reset the vertical speed on an impact.
- **Attitude error:** 2 degrees of it ruins every design's vertical estimate in hard banks, the baseline's trim included.
- **Full Speed from the hover** dips about 0.7 m/s in every design (project_plan.md, "Found while checking", point (1)).

When the discussion resumes, start from these patches and tables rather than re-deriving them.

## 2026-10-18: before the blue-sky rewrite (fallback)

Folder: `.bank-snapshots/2026-10-18-pre-rewrite/`. Plan: [plans/2026-10-17-blue-sky-rewrite.md](plans/2026-10-17-blue-sky-rewrite.md).

- **Before:** the owner said bank mode was "handling very well".
- **In the folder:** patches for both repos, the baseline `sim-bank.wasm`, and copies of the owner's logs 232749, 175027, 175231 and 151904. The copies survive a rebuild there; the sim's `testing-flight-logs/auto/` doesn't.
- **Tools:** the rewrite's evaluation tools (scorecard, ablation driver, ledger, design note) are in its `tools/` folder, and its README says how to run them.
- **Replays are chaotic:** compare builds on scripted scenarios, and on log segments only across perturbed ensembles.
- **Outcome:** the owner flew the rewrite on 2026-10-18 (`webfpv-bank-20261018-005929.csv`, copied into the snapshot's `logs/`) and said it feels better than before. The rewrite is the new base, and this snapshot is only a fallback. It supersedes the full-bank fixes snapshot below.

## 2026-10-17: full-bank fixes (older fallback)

Folder: `.bank-snapshots/2026-10-17-full-bank-fixes/`.

- **The state:** the owner flew the uncommitted full-bank fixes and said they feel ok.
- **Why it was saved:** as a revert point, at their request, before prototyping a track-direction estimate (least squares of drag and turn force).
- **In the folder:** patches for both repos, the flown `sim-bank.wasm` and the scratch harness `hardbank.mjs`.
