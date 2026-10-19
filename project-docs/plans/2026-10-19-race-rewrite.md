# 2026-10-19: race rewrite (branch `bank-race-rewrite`)

A record of the session's plan. What was built, measured and left open is in [project_plan.md](../../project_plan.md), "Race rewrite".

**Ask.** Rebuild bank mode from scratch as a racing mode that matches perfect acro in straight-line speed, full punch upwards and turn tightness. Work on an experimental branch, with the old code as intent only, and expect the mode to fly differently.

**Owner's answers before starting.**
- Full Bank is maximum sideways force, and Lift gives way.
- A hard turn brakes towards the corner speed.
- The Speed stick stays hybrid.
- Head check and look into the turn are noted as to-dos.
- Branch the sim too.

**Plan, as carried out.**
1. Branch both repos as `bank-race-rewrite`.
2. Write the mode around one thrust vector: Bank, then Lift, then Speed, with the corner speed from one airframe constant, `bank_drag_rate`. Add a velocity estimate that integrates the accelerometer and is corrected by the rotor drag. Rewrite the unit tests.
3. Build a yardstick: `npm run bank:bench` in the sim, bank mode against an ideal acro pilot on the three figures.
4. Iterate in the sim until all three reach about acro or better. Each fix followed a diagnosis from a trace, not a tuning sweep:
   - 3D velocity, so a climb isn't read as slip.
   - The trim measured along the thrust axis.
   - The rotor drag's lift and an integral on the measured vertical force.
   - A slow force budget.
   - Flying on rates instead of Euler angle targets.
   - The corner force before Bank.
   - Lift giving way by at most a fifth.
5. Replay the owner's latest real sticks in the air.
6. Bring the sim checks in line with the new spec, update the docs, and build SITL and the Air65 target.

**Left for the owner.**
- The two departures from the literal answers: Lift gives way by only a fifth, and the corner force comes before Bank.
- Whether to keep flying on rates.
- Fly it: `npm run serve` in the sim, open `/?fw=bank`, press B.
