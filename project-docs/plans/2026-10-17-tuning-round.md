# Plan: up to 15 loops of bank mode tuning and bug fixes

## Context

Bank mode works in the simulator and the owner says it feels ok at the committed state (`640173c32`, `bank_look_turn` default 0). `project_plan.md` lists the known rough edges, and the owner's two newest flights (`webfpv-bank-20261017-175027.csv`, `-175231.csv`) were flown on that build and haven't been read yet. This round fixes bugs and tunes the **current implementation**. It adds nothing new. The goal is a working tree, not committed, that the owner reviews and flies.

**Scope rules (owner, 2026-10-17)**
- Bug fixes and tuning only. No new settings, no new PG fields, no PG version bumps. Tuning means internal constants (such as `BANK_TILT_THRUST_SHARE`, filter rates, slew limits and fades) and the defaults of existing settings. Every changed default is listed in the report.
- The least-squares track estimate is out of scope. So are the look redesign, vertical speed controls and a total tilt limit setting.
- Decisions the plan marks as the owner's (the "Lift stays a force" law, thrust priority, look dot as the default, the threshold bands) are not reopened. Where a fix would need one of them, it goes in the report as a question.
- Don't commit in either repo. Don't change `tests/bank-thresholds.json`.
- Firmware changes are for flight behaviour only: no getters or debug fields added just for logging. Diagnostics use `DEBUG_BANK`, the existing `bankModeGet*` getters, sim plant state, or a scratch harness in the session scratchpad.

## Before loop 1

1. Read `$SIM_DIR/.claude/skills/bank-sim-check/SKILL.md` and `$SIM_DIR/BANK-MODE.md`, as the `bank-sim-loop` skill requires.
2. Baseline: `make test_bank_unittest`, then `npm run bank:cycle`. Save the table to the scratchpad. The known failing row at look 0 is `turn_ball_g` -0.112.
3. Read the newest owner logs first: `npm run bank:log` on `175027` and `175231`, with `--rows` on anything odd and `--replay` through the baseline build. Also note the replay metrics from the plan's earlier logs (`151904` 76–101 s, `004932`, `002232`) so later loops are compared against the same numbers.
4. Copy the scratch harness `hardbank.mjs` from `.bank-snapshots/2026-10-17-full-bank-fixes/` into the scratchpad as the scripted sortie set: medium bank 0.6 at 0.5 Speed, held full bank from 0.5, 0.75 and 1.0 Speed, full bank with Speed at the bottom, a reversal, and the owner's sticks 76–101 s. Run all of it at 1.62 g, from the ground.

## Candidate list (rank after the baseline, owner logs first)

Pick from the top. Anything new found in the owner's logs goes above these.

1. **Nose dive with Lift at the bottom.** `bankForceToTilt()` divides the forward force by a lift near zero, so the pitch target goes to 80–85°. A bug fix inside the current law: bound the forward tilt when there is no lift to tilt, for example by holding it or limiting it by the lift. No setting.
2. **`core.c` I-term reset with the throttle stick low.** Under BANK the throttle stick is the Speed command, so braking resets the I-term. Add a BANK exception. This is a correctness bug that has to be fixed before hardware.
3. **Yaw saturation flip-flop at low speed.** In hard banks at 8–14 m/s, and while sinking, the yaw flips between +360 and −360 deg/s. Look at the feedforward fade, and at how the ball and the turn-gain learner gate on saturation.
4. **Held full bank at look 0 speeds up** (17→27 m/s at 0.5 Speed, with the ball 15–28° off the track). Tune within the current design: the coordinated-speed cap and the forward-tilt share beside a steep bank.
5. **Reversal rock**: nose-up pitch for 0.3 s and the look swinging at 360 deg/s. Tune only what is already there (look slew, rolling gate). Don't repeat the reverted look-timing experiments listed in the plan.
6. **Slow sink at full Speed** (trim leak, about −2.6 m/s after 10 s at 40 m/s): tune the trim's leak and rates.
7. **Head check moves the track by about 10°** (`headcheck_track_change_deg` −11.1 at 0.27 Speed): the tilt is rotated by the commanded offset while the nose lags it.
8. **Mid-stick speed short** (0.43 g against 0.50 g of drag): the deadband and the P-only loop. Tune within the existing `bank_speed_kp` and deadband defaults.
9. **`BANK_TILT_THRUST_SHARE` 0.75**: A/B 0.70/0.80 for roll-over margin against top speed. The value is tuned only, not made a setting.

## Each loop (max 15, stop early when the list is done or results flatten)

1. Pick one item. Say what is wrong and which numbers should move: a log row, a replay or a harness sortie.
2. Make the smallest change, in `src/main/flight/bank_mode.c` or the existing hooks, in Betaflight style.
3. Add or adjust a unit test in `src/test/unit/bank_unittest.cc` when the bug can be tested. Run `make test_bank_unittest`.
4. `npm run bank:cycle` in `$SIM_DIR`. Handle DRIFT or compile failures as the skill says (port to `patches/bank/`, `compat.sed`). Don't change firmware to suit 4.5.1.
5. Re-run the harness set and the owner-log replays. Compare against the baseline and the previous loop.
6. **Keep or revert.** Keep the change only if its target improves and no other row, sortie or replay gets clearly worse, with "flight controls first" checked for anything that touches the look. Otherwise revert it with `git checkout` on the edited hunks. Revert only my own edits: the tree is clean at the start.
7. Note the loop in a scratchpad log: item, change, before and after numbers, kept or reverted.

After every 5 loops: `make test` (the full non-target suite plus the pg-id check) and `make TARGET=SITL`, so the change set still builds outside the sim.

## Wrap-up

- Update `project_plan.md`: move fixed items out of "To look at" and "Known open points", record measured numbers and anything tried and reverted, in the owner's terms.
- Final checks: `make test_bank_unittest`, `make test`, `make checks`, `make TARGET=SITL`, `npm run bank:cycle` (full table), and the harness set at look 0 and 100.
- Report to the owner: a per-loop table (kept or reverted, with the numbers), any changed defaults or constants, open questions that need their decisions, and what to fly by hand (`npm run serve`, `/?fw=bank`) to judge feel. Point out that the sim checkout lives under `/home/devpod`, so a rebuild loses it until it is pushed.

## Critical files

- `src/main/flight/bank_mode.c` (most fixes), `src/main/flight/bank_mode.h`
- `src/main/fc/core.c` (I-term reset exception), `src/main/flight/pid.c` (only if a loop needs it)
- `src/test/unit/bank_unittest.cc`
- `project_plan.md` (wrap-up)
- Sim, only if a hook changes: `$SIM_DIR/patches/bank/`, then `bash scripts/bank-sync.sh --accept`
