---
name: bank-sim-loop
description: Test bank flight mode changes end to end in the WebFPVSimulator. Use after any change to src/main/flight/bank_mode*, src/main/pg/bank_mode*, or a BANK_MODE hook (rc.c, pid.c, mixer.c, core.c, rc_modes.h, runtime_config.h), before saying a bank mode change works, and when asked to fly, simulate or check bank mode.
---

# Bank mode: firmware edit to simulated flight

The simulator lives at `$SIM_DIR` (default `/home/devpod/WebFPVSimulator`). It compiles Betaflight 4.5.1, not this branch, and overlays the bank mode from this working tree, uncommitted edits included. The full mechanics and rules are in the sim's own skill, `$SIM_DIR/.claude/skills/bank-sim-check/SKILL.md`. Read it before the first iteration in a session. The owner's how-to for running and flying it is `$SIM_DIR/BANK-MODE.md`.

## The loop

1. Edit the firmware here. Keep `bank_mode.c` to inputs a real flight controller has (gyro, acc, attitude, RC, arm state).
2. `make test_bank_unittest` (seconds). Fix before going further.
3. `cd $SIM_DIR && npm run bank:cycle`. This syncs, builds `dist/sim-bank.wasm`, checks the vendor tree, flies the scripted sortie and prints a table against bands taken from `project_plan.md`.
4. If it reports **DRIFT**, you added, removed or changed a hook in a file the sim compiles. Port the same change to the sim's 4.5.1 patch, as the sim skill describes, then `bash scripts/bank-sync.sh --accept`.
5. If it fails to **compile**, `bank_mode.c` is using a master-only API. Add a mapping to `patches/bank/compat.sed` or a define in `scripts/build-wasm.sh`. Do not change the firmware to suit 4.5.1.
6. Read failing rows with `npm run check:bank -- --explore`. Fix the firmware, then repeat from 2.
7. If the sim repo changed, commit and push it. There is no volume, so a container rebuild re-clones it.

## What a pass means

The harness checks behaviour the spec promises (zero vertical acceleration at lift centre, ball centred, coordinated turn, head check keeps the track, braking, determinism). It runs on the plant's true attitude and a clean accelerometer, so it is optimistic. Report the table, the known limits, and leave feel to the owner: `npm run serve` in the sim, open `/?fw=bank`, press B in flight.

Do not change `tests/bank-thresholds.json` to get a pass. If a band contradicts the spec or physics, say so and let the owner decide.
