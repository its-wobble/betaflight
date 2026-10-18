# Brake before Bank

## Context

Thrust priority is Lift, then Bank, then Speed (`bankThrustVector()`, [bank_mode.c:219-225](src/main/flight/bank_mode.c#L219-L225)). Full Bank asks for 90 degrees, so it takes the whole horizontal budget and leaves `forwardMax` ≈ 0. A brake asked for in a hard bank then comes out at 0. The owner's logs confirm it. In `webfpv-bank-20261018-211048.csv`, 18 % of the BANK frames (1748 of 9645) have the speed loop asking for −0.5 to −1.05 g of forward force (DEBUG_BANK 3) at ≥ 0.6 Bank stick, and the pitch target sits at 0 (13.7 s, 43.5 s, 53.6 s, at 15 to 29 m/s). The same shows in 155317, 171544, 171857, 205131 and 210138.

**Owner's decision (2026-10-18):** while braking, the order is **Lift, Speed, Bank**. Lift is still kept first. When not braking, nothing changes.

## Change

**[bank_mode.c](src/main/flight/bank_mode.c) `bankThrustVector()`**: take the brake out of the budget before the bank.

```c
const float brake = fminf(fmaxf(-forwardForce, 0.0f), fminf(lift * tanf(DEGREES_TO_RADIANS(BANK_MAX_BRAKE_DEG)), horizontal));
const float bankMax = atan2f(sqrtf(fmaxf(sq(horizontal) - sq(brake), 0.0f)), lift);
const float bank = fminf(fabsf(bankStick) * M_PIf / 2.0f, bankMax);
*right = copysignf(lift * tanf(bank), bankStick);
const float forwardMax = sqrtf(fmaxf(sq(horizontal) - sq(*right), 0.0f));
*forward = forwardForce < 0.0f ? -brake : fminf(forwardForce, forwardMax);
```

- "Braking" means a negative forward force. Only the bottom third of the Speed stick (a craft faster than the target) or the reverse zone gives one, because above the third the P term only adds force.
- The brake keeps its 30-degree limit, so in a bank it is the same force as a level brake: at most `lift * tan 30` (0.58 g at 1 g of Lift), and never more than the budget.
- It is continuous in the forward force: as the brake goes to 0, `bankMax` goes back to the full budget, so the bank does not jump when the stick crosses the third.
- Cost to the bank: 77.2 to about 76.9 degrees at T/W 5.7, and 65.2 to 64.4 degrees at trim 0.5. The total tilt budget is unchanged (|thrust| ≤ Tb still), so the collective floor at [bank_mode.c:382-384](src/main/flight/bank_mode.c#L382-L384) needs no change.
- Also counted as braking: the reverse (≤ about 0.15 g), which takes a negligible share of the bank.

**Comments**: update the header's priority sentence ([bank_mode.c:69-70](src/main/flight/bank_mode.c#L69-L70)) and the function comment ([bank_mode.c:216-218](src/main/flight/bank_mode.c#L216-L218)) to say that Bank takes what it needs after a brake. Keep them short, with no flight-test history.

**Unit test, [bank_unittest.cc](src/test/unit/bank_unittest.cc)**:
- In `BankTakesTheHorizontalForceBeforeSpeed` (line 176), replace the last case, a full bank with braking giving 0, with a new test `ABrakeTakesTheHorizontalForceBeforeBank` that checks:
  - full Bank with `forwardForce = -10`, lift 1, horizontal 3: forward = −tan 30, right = sqrt(9 − tan² 30);
  - a small brake (−0.1) leaves the bank nearly the full budget (continuity);
  - a brake larger than the budget (lift 2, horizontal 0.5) gives forward = −0.5 and right = 0;
  - a positive force in full Bank still gives forward ≈ 0 (Bank before Speed when not braking).
- Add an end-to-end case after `ForwardDragReadsAsSpeedAndTheBottomOfTheStickBrakes` (line 479): the same inputs plus `in.roll = 1.0f`. Check that the pitch target is clearly negative (tilted back) and the roll target is still near the full-bank angle.

**[project_plan.md](project_plan.md)**:
- Update "Thrust priority" in Terms (line 36).
- Update the Rewrite design bullet (line 60).
- Add a dated entry: "Brake before Bank (owner, 2026-10-18)", with the log evidence and the measurements from below.

Afterwards, copy this plan to `project-docs/plans/2026-10-18-brake-before-bank.md`.

## Verification

1. `make test_bank_unittest`.
2. `cd $SIM_DIR && npm run bank:cycle`. Compare the table with the last run: the brake rows should hold, and the turn rows shouldn't move, since the sortie doesn't brake in a bank.
3. Replay the owner's sticks through the new build, `npm run bank:log -- <211048 log> --from 10 --replay` (and 171857 around 159 to 175 s). Before and after, compare the pitch target and the speed decay where the brake meets full Bank, and check for no new roll-target swings or yaw saturation at the brake transitions.
4. Report the results and leave the feel to the owner (`npm run serve`, `?fw=bank`). No commit until they ask.
