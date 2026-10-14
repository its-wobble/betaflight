# AGENTS.md

Betaflight is C flight-controller firmware for many MCU families (STM32, AT32, APM32, ESP32, RP2/PICO, plus a Linux SITL simulator), built with GNU make and the Arm GNU toolchain. Unit tests are Google Test (C++) in `src/test`.

## Project context

The current work is a **new flight mode, "bank"** (spec and plan in [project_plan.md](/workspaces/betaflight/project_plan.md)). It is a personal project that may be upstreamed one day, so develop it the Betaflight way: small single-purpose changes, no target-specific code in `src/main`, compile-time feature gating, unit tests, and PRs against `master` (see Conventions).

**Testing in the simulator**: the bank mode is flown end to end in the WebFPVSimulator (`/home/devpod/WebFPVSimulator`, cloned by `.devcontainer/setup-sim.sh`). After any bank mode change, follow the `bank-sim-loop` skill (`.claude/skills/bank-sim-loop/SKILL.md`): `make test_bank_unittest`, then `npm run bank:cycle` in the sim. How to run and fly it by hand: `BANK-MODE.md` in the sim repo.

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
```

- `TARGET` and `CONFIG` are mutually exclusive. Building a `CONFIG` fails until `make configs` has been run.
- Heavy vendor SDKs (pico-sdk, esp-idf, STM32H5/N6/C5, APM32F4) are opt-in: `make SDK=<name> platform-sdk-hydrate` (`make platform-sdk-list-print` lists them).
- Per-developer make overrides go in `mk/local.mk` (gitignored).

### Unit tests (`src/test`)

```bash
make test                         # non-target-specific tests
make test-all                     # plus every per-target "expanded" test (CI: make EXTRA_FLAGS=-Werror checks test-all)
make test_help                    # list available tests
make test_pg_unittest             # build + run ONE test: test_<basename of src/test/unit/<name>.cc>
./obj/test/pg_unittest/pg_unittest --gtest_filter='Suite.Case'   # one gtest case, after building
```

CI needs `clang-18` and `libblocksruntime-dev` for the tests.

To add a test, create `src/test/unit/<name>_unittest.cc` **and** define `<name>_unittest_SRC` in [src/test/Makefile](/workspaces/betaflight/src/test/Makefile) (optional: `_DEFINES`, `_INCLUDE_DIRS`, `_LDFLAGS`, `_EXPAND`, `_BLACKLIST`); a test without `_SRC` is a hard make error. Tests link only the listed production `.c` files and stub everything else, so new dependencies in the unit under test need extra sources or stubs. Features are enabled per test with `USE_FOO=` entries in `_DEFINES`. `make test` also runs `check-pg-ids` against `src/main/pg/pg_ids.h`.

`src/test/sitl/sitl_harness.py` is an end-to-end SITL scenario runner (UDP RC/FDM feeds + MSP); useful for exercising a mode without hardware.

## Architecture

- **`src/main/`** – platform-independent logic: `fc/` (core loop, tasks, rc/modes), `flight/` (PID, mixer, imu, alt/pos hold, GPS rescue), `sensors/`, `drivers/`, `io/`, `rx/`, `telemetry/`, `osd/`, `blackbox/`, `msp/`, `cli/`, `scheduler/`, `pg/` (parameter groups), `common/`, `target/` (shared `common_pre.h` / `common_post.h` / `common_defaults_post.h` feature headers).
- **`src/platform/<PLATFORM>/`** – everything MCU-specific (drivers, DMA, timers, linker scripts, per-family `mk/`, and `target/<MCU>/target.{h,mk}` setting `TARGET_MCU`, `TARGET_MCU_FAMILY`, source lists). `SIMULATOR/target/SITL` is the simulator "MCU".
- **Board configs** live in the separate `betaflight/config` repo (the `src/config` submodule), not here.
- **Build system**: top-level [Makefile](/workspaces/betaflight/Makefile) includes `mk/*.mk` and the selected platform's `mk/*.mk` + `target.mk`; sources are found via `VPATH` over `src/main`, `lib/main`, `lib/modules`, `src/platform`. `lib/` is vendored third-party code.
- **Compile-time features**: capabilities are gated by `USE_*` macros defined in `src/main/target/common_*.h`, platform `target.h` and board `config.h`. Code must build with a feature both on and off.
- **Parameter groups**: persistent config is registered with `PG_REGISTER*` macros (linker section `.pg_registry`). Each PG needs a unique ID in `src/main/pg/pg_ids.h` and a version bump when its struct layout changes. CLI settings are exposed in `src/main/cli/settings.c`.
- **Scheduler**: work runs as scheduler tasks (`src/main/scheduler/scheduler.c`, task table in `src/main/fc/tasks.c`); the gyro/PID loop is the highest-rate task. Hot-path variables use `FAST_DATA` (non-zero initialiser) or `FAST_DATA_ZERO_INIT` (zero/default initialiser); `make checks` enforces the right one.
- **Flight modes**: a box ID (`fc/rc_modes.h`, `boxId_e`) is mapped to a bit in `flightModeFlags` (`fc/runtime_config.h`, `BOXID_TO_FLIGHT_MODE_MAP_INITIALIZER`), advertised to configurators in `msp/msp_box.c`, and queried with `FLIGHT_MODE(X_MODE)` / `IS_RC_MODE_ACTIVE(BOXX)`. `flightModeFlags` is `uint16_t`; bits 0–13 are used (4 and 9 are commented-out), so 14 and 15 are free.

## Conventions

- **Target independence**: code in `src/main` (outside `src/main/target`) must not mention MCU/board target names (`make checks` greps for it). Put hardware specifics under `src/platform/` and gate with `USE_*`.
- Follow the [coding style](https://www.betaflight.com/docs/development/CodingStyle); new source files carry the GPL header used across `src/main`.
- PRs target `master`, stay small and single-purpose (never combine a feature with a new target), ideally one descriptive commit; reference issues with `Fixes #<n>`; run `make preview` first. See [.github/pull_request_template.md](/workspaces/betaflight/.github/pull_request_template.md). Board/target PRs belong in `betaflight/config`.
- Developer docs: https://betaflight.com/docs/development. A devcontainer with toolchain, DFU and OpenOCD is in `.devcontainer/`.
