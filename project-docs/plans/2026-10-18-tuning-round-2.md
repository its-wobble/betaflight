# Tuning round 2 (2026-10-18)

Owner's brief: up to 20 loops of tuning and bug fixes. Don't commit. Don't increase complexity significantly: small extra logic is fine, no systems for edge cases; ideally fixes fall out of better algorithms. No new features, including items from "To look at", unless it's a bug fix. Ultimately for a real drone.

How it ran (no separate plan was written; this is the record):
1. Baseline: `make test_bank_unittest`, `npm run bank:cycle`, and the owner's newest flights (212543 was flown with brake before Bank). Their complaint, braking in a hard bank, reproduced from the log.
2. A scratch scorecard (the rewrite's, brought up to date) for each change: five inch at 1.62 g from the ground, level variants, whoop65, owner segments with perturbed ensembles, whole flights at five pack voltages, look turn 0 and 100.
3. Keep a change only if its target moved and nothing else got clearly worse; otherwise revert. Every kept change has a unit test that fails without it, where the behaviour is testable.
4. After loop 10 and at the end: `make test`, `make checks`, `make TARGET=SITL`, `make CONFIG=BETAFPVG473_V2 OPTIONS="USE_BANK_MODE"`.

Loops (kept: 2, 3, 6, 7, 11 and the test of 13; the rest reverted or no change). Details and numbers are in project_plan.md, "Tuning round 2", and loop by loop in `.bank-snapshots/2026-10-18-tuning-round-2/loops.md`.

1. Thrust along the drag-measured track: reverted (vertical speed corrupts the track at steep bank).
2. Ball integral: kept.
3. core.c reads the collective for the airmode latch, runaway takeoff and launch control: kept.
4. Trim filter 5 Hz: reverted (ground spin-up).
5. Bank held to the yaw limit's share: reverted.
6. Yaw limit 720, nose slew 360: kept.
7. Ball faded out with the speed: kept.
8. Forward force waits for the measured bank: reverted (the exit's nose swing is the angle loop's Euler coupling).
9. Ki 300/600/1200: 600 kept.
10. Hook review (mixer, rc, failsafe, core): nothing found.
11. Drag filtered in the travel frame: kept.
12. Integral bound: no evidence of harm, not added.
13. Unit test for 11.
14. Braking balloon from level flight: 1.4 m/s, no change.
15. Ground spin-up trim (1.29 for about a second): the same as before, no change.
16. Look turn 100: the same crashes as before, no change.
