# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Betaflight is C flight-controller firmware for STM32, AT32, APM32, ESP32 and RP2 MCUs, plus a Linux SITL simulator. It builds with GNU make and the Arm GNU toolchain. Unit tests are Google Test (C++) in `src/test`.

## Bank mode (the current work)

This fork (its-wobble/betaflight) adds a new flight mode, **bank**. The spec, decisions and open points are in [project_plan.md](project_plan.md). Use the owner's terms from its "Terms" section: **Speed**, **Lift** and **Bank** are the controls, in the world frame; **thrust** is the props' force, in the drone frame.

The mode lives in `src/main/flight/bank_mode.{c,h}` and `src/main/pg/bank_mode.{c,h}`, gated by `USE_BANK_MODE` (`target/common_post.h`), with hooks in `fc/rc.c`, `fc/core.c`, `flight/pid.c`, `flight/mixer.c`, `msp/msp_box.c`, `osd/osd_elements.c` and `cli/settings.c`. It is a personal project that may be upstreamed, so develop it the Betaflight way (see Conventions).

- **Don't commit** in this repo or the sim unless the owner asks. They review changes first.
- **Test every bank mode change in the simulator** with the `bank-sim-loop` skill: `make test_bank_unittest`, then `npm run bank:cycle` in `$SIM_DIR` (`/home/devpod/WebFPVSimulator`). The owner's guide to flying it by hand is `$SIM_DIR/BANK-MODE.md`.
- **Read the owner's flight log before guessing** at a problem they report. When they fly `?fw=bank` (`npm run serve` in the sim), the page posts the flight to `$SIM_DIR/testing-flight-logs/auto/` once a second, with the firmware's targets, collective, thrust to weight and `DEBUG_BANK` fields. In the sim, `npm run bank:log` summarises the newest log, `--rows FROM TO` dumps frames, and `--replay` flies the logged sticks headless through the current build. Scripted sorties have twice missed what the owner's sticks provoked. The folder is gitignored and is lost on a container rebuild.
- **The sim flies at 1.62 g.** The five inch's Weight 100 sets that, and the sim's accelerometer reads in local g, so a hover still reads 1 g. A check flown at the module's default 1.0 g, or in free air rather than from the ground, does not see what the owner flies. `bank-check.js` and `bank-log.js --replay` already set both.
- **Real hardware**: the owner's test drone is a BetaFPV Air65 R (`BETAFPVG473_V2`, 512 KB G473, BMI270, 1S whoop). Read [air65-backup/reference.md](air65-backup/reference.md) before any hardware work: it has the restore steps, the build (`make CONFIG=BETAFPVG473_V2 OPTIONS="USE_BANK_MODE"`, since the flash gate leaves 512 KB parts out), the bench test and the log fields. The open points are in project_plan.md's "Real hardware". The container has no USB, so the owner flashes and pulls logs on their own computer, following [air65-backup/flashing.md](air65-backup/flashing.md). Full chip erase is safe: the board's crystal works, though the stock settings run it on the internal clock (`system_hse_mhz = 0`).
- **Firmware changes are for flight behaviour only.** Don't add getters, outputs or debug fields to `bank_mode.c` just to feed logging. Log from `DEBUG_BANK`, the existing `bankModeGet*` getters or the sim's own plant state. If a diagnostic needs firmware internals, use a temporary unit-test harness, or ask first.

**Dev environment**: `/workspaces/betaflight` survives a container rebuild. Nothing under `/home/devpod` does, including the sim checkout and Claude's memory, so push sim work before a rebuild. If the container, the sim checkout, the env vars (`$SIM_DIR`, `$BF_BANK_SRC`, `$SIM_CHROME_BIN`) or the headless browser (Xvfb on `:99`) breaks, follow [.devcontainer/SIM-SETUP.md](.devcontainer/SIM-SETUP.md).

## Build, test, lint

Run from the repo root.

```bash
make arm_sdk_install            # install the pinned Arm toolchain into ./tools (once)
make configs                    # hydrate board configs (src/config submodule = betaflight/config repo)

make TARGET=STM32F405           # MCU target (dirs under src/platform/<PLATFORM>/target/)
make CONFIG=MATEKF405           # board config (src/config/configs/[<mfr>/]<BOARD>/config.h)
make TARGET=SITL                # Linux software-in-the-loop (obj/main/betaflight_SITL.elf)
make V=1 ...                    # verbose; DEBUG=GDB|INFO for debug builds; OPTIONS="USE_FOO ..." for extra defines
make targets                    # list valid targets; `make help` lists everything

make checks                     # sanity checks in mk/checks.mk (target independence, FAST_DATA usage, ...)
make preview                    # representative subset of the CI build + tests; run before opening a PR
make cppcheck                   # static analysis

make test                         # non-target-specific unit tests (also runs check-pg-ids)
make test-all                     # plus every per-target "expanded" test (CI: make EXTRA_FLAGS=-Werror checks test-all)
make test_help                    # list available tests
make test_bank_unittest           # build + run ONE test: test_<basename of src/test/unit/<name>.cc>
./obj/test/bank_unittest/bank_unittest --gtest_filter='Suite.Case'   # one gtest case, after building
```

- `TARGET` and `CONFIG` are mutually exclusive. Building a `CONFIG` fails until `make configs` has been run.
- Heavy vendor SDKs (pico-sdk, esp-idf, STM32H5/N6/C5, APM32F4) are opt-in: `make SDK=<name> platform-sdk-hydrate` (`make platform-sdk-list-print` lists them).
- Per-developer make overrides go in `mk/local.mk` (gitignored).
- **Adding a unit test**: create `src/test/unit/<name>_unittest.cc` **and** define `<name>_unittest_SRC` in [src/test/Makefile](src/test/Makefile) (optional: `_DEFINES`, `_INCLUDE_DIRS`, `_LDFLAGS`, `_EXPAND`, `_BLACKLIST`). A test without `_SRC` is a hard make error. Tests link only the listed production `.c` files and stub everything else, so a new dependency in the unit under test needs another source or a stub. Enable features per test with `USE_FOO=` entries in `_DEFINES`.
- `src/test/sitl/sitl_harness.py` runs end-to-end SITL scenarios (UDP RC/FDM feeds + MSP), for exercising a mode without hardware.

## Architecture

- **Platform split**: `src/main/` is platform independent. Everything MCU specific (drivers, DMA, timers, linker scripts) is under `src/platform/<PLATFORM>/`, with `target/<MCU>/target.{h,mk}` setting `TARGET_MCU`, `TARGET_MCU_FAMILY` and source lists. `SIMULATOR/target/SITL` is the simulator "MCU". Board configs live in the separate `betaflight/config` repo (the `src/config` submodule).
- **Build system**: the top-level [Makefile](Makefile) includes `mk/*.mk` and the selected platform's `mk/*.mk` + `target.mk`. Sources are found via `VPATH` over `src/main`, `lib/main`, `lib/modules` and `src/platform`; new `src/main` files are listed in `mk/source.mk`. `lib/` is vendored third-party code.
- **Compile-time features**: capabilities are gated by `USE_*` macros from `src/main/target/common_*.h`, the platform `target.h` and the board `config.h`. Code must build with a feature both on and off.
- **Parameter groups**: persistent config is registered with `PG_REGISTER*` macros (linker section `.pg_registry`). Each PG needs a unique ID in `src/main/pg/pg_ids.h` and a version bump when its struct layout changes. CLI settings are exposed in `src/main/cli/settings.c`.
- **Scheduler**: work runs as scheduler tasks (`scheduler/scheduler.c`, task table in `fc/tasks.c`); the gyro/PID loop is the highest-rate task. Hot-path variables use `FAST_DATA` (non-zero initialiser) or `FAST_DATA_ZERO_INIT` (zero or default initialiser); `make checks` enforces the right one.
- **Flight modes**: a box ID (`fc/rc_modes.h`, `boxId_e`) maps to a bit in `flightModeFlags` (`fc/runtime_config.h`, `BOXID_TO_FLIGHT_MODE_MAP_INITIALIZER`), is advertised to configurators in `msp/msp_box.c`, and is queried with `FLIGHT_MODE(X_MODE)` / `IS_RC_MODE_ACTIVE(BOXX)`. `flightModeFlags` is `uint16_t`: bits 0–13 are upstream modes (4 and 9 commented out), 14 is `BANK_MODE`, so only 15 is free.

## Conventions

- **Target independence**: code in `src/main` (outside `src/main/target`) must not mention MCU or board target names; `make checks` greps for them. Put hardware specifics under `src/platform/` and gate them with `USE_*`.
- Follow the [coding style](https://www.betaflight.com/docs/development/CodingStyle). New source files carry the GPL header used across `src/main`.
- **Comments**: match the rest of `src/main` (`flight/alt_hold_multirotor.c` and `flight/autopilot_multirotor.c` are good references). Comment what a reader can't see from the code: the reason for a non-obvious line, units, sign conventions, a formula. Keep it to a line or two, and leave the obvious uncommented. Leave out flight-test history: log file names, simulator measurements, and what went wrong before a fix. That goes in the commit message or `project_plan.md`. This applies to the bank mode code too, which uses the terms from `project_plan.md`.
- PRs target `master`, stay small and single-purpose (never combine a feature with a new target), ideally as one descriptive commit. Reference issues with `Fixes #<n>` and run `make preview` first. See [.github/pull_request_template.md](.github/pull_request_template.md). Board and target PRs belong in `betaflight/config`.
- **Fork-only files**: `CLAUDE.md`, `.claude/skills/`, `project_plan.md`, `.vscode/tasks.json`, `air65-backup/`, `.devcontainer/SIM-SETUP.md` and the sim additions to `.devcontainer/` exist for the bank mode work. Drop them, and restore upstream's `.gitignore` block, before opening a PR against betaflight.
