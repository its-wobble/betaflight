# BetaFPV Air65 R: stock backup and restore reference

Written 2026-10-18, before the drone was first flashed with custom (bank mode) Betaflight firmware. It records what the drone is, what was backed up, how the backup was made, and how to restore it. It is meant to be read by the owner or handed to an LLM for context, together with the files listed under "Backup set".

## Backup set

Keep these files together, with a second copy somewhere else.

| File | What it is | How it was made |
|---|---|---|
| `air65_stock_full.bin` | Byte-for-byte image of the flight controller's whole internal flash (512 KB = 524288 bytes): the stock firmware **and** its saved settings | Read over USB DFU with `dfu-util` (see "How the flash image was made"). Read twice; both reads had matching SHA-1 hashes |
| `BTFL_cli_backup_AIR65_R_20261018_130941_BETAFPVG473_V2.txt` | `diff all`: every setting that differs from the defaults. Can be pasted into the CLI to restore the settings | Betaflight App backup |
| *(owner's `dump all` file)* | `dump all`: every setting, defaults included. **Reference only. Do not paste it into another firmware version** | CLI `dump all` |
| this file | Context and procedures | |

SHA-1 of `air65_stock_full.bin` (fill in with `sha1sum air65_stock_full.bin`): `________________________________________`

## The drone

| Item | Value |
|---|---|
| Craft | BetaFPV Air65 R, 65 mm ducted whoop, 1S (craft name `AIR65 R`) |
| Flight controller board | `BETAFPVG473_V2`, manufacturer ID `BEFH` |
| MCU | STM32G474 family, reports as `STM32G47X (G473)`, 512 KB flash |
| MCU ID | `002e00253235511135363938` |
| DFU serial | `3564395D3235` |
| Gyro / accelerometer | **BMI270** (confirmed by `status`), on SPI1 with DMA |
| OSD | MAX7456 analog OSD (30 x 13), NTSC |
| Onboard blackbox flash | JEDEC ID `0x00852018`, 16 Mbit (2 MB) |
| Receiver | ExpressLRS, serial CRSF on UART3. A separate chip with its own firmware and binding |
| ESCs | 4-in-1 on the board with their own firmware (not touched by FC flashing). Bidirectional DShot300, 12 pole motors, `yaw_motors_reversed = ON` (props out) |
| Board orientation | `align_board_yaw = -135` |
| System clock | **168 MHz from the internal oscillator (PLLR-HSI)**, because the settings contain `set system_hse_mhz = 0`. See "Clock caveat" |

### The IMU substitution card

The drone came with a card saying the FC may use any of these gyros because of chip shortages: ICM42688, ICM42622, BMI270, LSM6DSV16X, LSM6DSK320X. It said not to flash an official Betaflight *release*, because releases at the time did not support some of these chips, and to use the development App at master.app.betaflight.com instead. It also gave this OSD fix, which is already in the settings: `set osd_displayport_device = MAX7456`, `set vcd_video_system = NTSC`, `save`.

What was found:

- This drone has a **BMI270**: there is a sticker on the board, and `status` reports it.
- The `BETAFPVG473_V2` board config (betaflight/config repo, `configs/BEFH/BETAFPVG473_V2/config.h`) enables BMI270, ICM42622P, ICM42688P and LSM6DSK320X. It does **not** enable the LSM6DSV16X. Upstream Betaflight has an LSM6DSV16X driver, which a build could add with `OPTIONS="USE_ACCGYRO_LSM6DSV16X"`.
- The "customized" stock firmware is plain upstream Betaflight development (master) firmware built with that board config by Betaflight's cloud build server (it has a BUILD KEY). It is not a BetaFPV fork.

## Stock firmware

```
# Betaflight / STM32G47X (G473) 2026.6.0-alpha May 15 2026 / 06:14:55 (e92c10887) MSP API: 1.48
# config rev: 6703db5
# board: manufacturer_id: BEFH, board_name: BETAFPVG473_V2
BUILD KEY: 3fcfddcd33eae651632dee98e6eeb2f5 (2026.6.0-alpha)
```

- `e92c10887` is upstream betaflight/betaflight master commit "Updating config submodule reference [15-05-2026]". It pins the config submodule at `6703db5c3e570110f22c5d88507fc6d2616bb1aa`.
- Being a cloud build, it contains only the feature options that were selected when it was built. A local `make CONFIG=BETAFPVG473_V2` build of the same commit has Betaflight's default feature set, so it is functionally close but not byte-identical.

### `status` output on the stock firmware (USB only, no battery)

```
MCU: STM32G474xx CLK=168MHz (PLLR-HSI), Vref=3.31V, Core temp=49degC
STACK: 2048b (0x2001fff0)
CONFIG: CONFIGURED (4109b / 8192b)

DEVICES DETECTED: SPI=1, I2C=0 (0 errors)
GYRO: (1) BMI270 enabled locked dma
ACC: BMI270
OSD: MAX7456 (30 x 13)
FLASH: JEDEC ID=0x00852018 16M

BUILD KEY: 3fcfddcd33eae651632dee98e6eeb2f5 (2026.6.0-alpha)

System Uptime: 74 seconds, Current Time: 2026-10-18T02:55:41.131+00:00
CPU:39%, cycle time: 309, GYRO rate: 3236, RX rate: 251, System rate: 9
Voltage: 0.74V (0S battery - NOT PRESENT)
Arming disable flags: CLI MSP DSHOT_TELEM
```

The arming disable flags are normal with the CLI open and no battery. The gyro runs at 3.2 kHz (BMI270) and the receiver at 250 Hz.

## Notable settings (from the `diff all`)

The full list is in the backup file. These are the ones that matter for a restore or are easy to lose:

- `system_hse_mhz = 0`: run from the internal oscillator. The board config's default is an 8 MHz crystal (`SYSTEM_HSE_MHZ 8`). BetaFPV changed this deliberately.
- `acc_calibration = -23,35,26,1`: accelerometer calibration. Losing it means recalibrating on a level surface.
- `osd_displayport_device = MAX7456`, `vcd_video_system = NTSC`: the card's OSD fix.
- Motors: `motor_pwm_protocol = DSHOT300`, `dshot_bidir = ON`, `motor_poles = 12`, `yaw_motors_reversed = ON`.
- `align_board_yaw = -135`, `vbat_scale = 113`, `ibata_offset = -600`, `small_angle = 180`.
- PID profile 0 is named `GF 1207` (BetaFPV's tune). Rate profile 0 uses `roll_srate`/`pitch_srate` 58 and `yaw_srate` 50.
- VTX: band 5 (Raceband) channel 1, 5658 MHz, power index 1 (25 mW). The VTX table is in the diff.

Mode switches (`aux <slot> <mode permanent ID> <AUX channel, 0 = AUX1> <low> <high> ...`):

| Line | Mode | Switch |
|---|---|---|
| `aux 0 0 0 1700 2100 0 0` | ARM | AUX1 high |
| `aux 1 1 1 900 1300 0 0` | ANGLE | AUX2 low |
| `aux 2 2 1 1300 1700 0 0` | HORIZON | AUX2 middle |
| *(none)* | acro | AUX2 high |
| `aux 4 35 2 1700 2100 0 0` | FLIP OVER AFTER CRASH (turtle) | AUX3 high |
| `aux 3 13 3 1700 2100 0 0` | BEEPER | AUX4 high |

## How the flash image was made

On the owner's computer (CachyOS, an Arch-based Linux), not in the dev container, which has no USB access. Reading does not change anything on the drone: `-U` (upload) copies device to computer, and only `-D` (download) writes.

1. Install dfu-util: `sudo pacman -S dfu-util`. On other systems: `brew install dfu-util` (Mac), `sudo apt install dfu-util` (Debian/Ubuntu), or the Windows binaries from dfu-util.sourceforge.net plus the WinUSB driver for "STM32 BOOTLOADER" via Zadig.
2. Enter the STM32 ROM bootloader (no battery needed, USB power only). Either connect in the Betaflight App, open the CLI and type `bl`, or hold the BOOT button while plugging in USB. Then close the Betaflight App.
3. Check the device: `dfu-util -l`. The listing seen on this drone:
   ```
   Found DFU: [0483:df11] ver=0200, ... alt=2, name="@OTP Memory   /0x1FFF7000/01*1024 e", serial="3564395D3235"
   Found DFU: [0483:df11] ver=0200, ... alt=1, name="@Option Bytes   /0x1FFF7800/01*048 e/0x1FFFF800/01*048 e", serial="3564395D3235"
   Found DFU: [0483:df11] ver=0200, ... alt=0, name="@Internal Flash   /0x08000000/256*02Kg", serial="3564395D3235"
   ```
   `256*02Kg` is 256 pages of 2 KB = 512 KB of internal flash at `0x08000000`.
4. Read the whole flash:
   ```bash
   dfu-util -a 0 -d 0483:df11 -s 0x08000000:524288 -U air65_stock_full.bin
   ```
5. Check it: `stat -c %s air65_stock_full.bin` prints `524288`. `hexdump -C air65_stock_full.bin | head -3` shows real data, not all `ff`. A second read into another file has the same `sha1sum`.
6. Unplug and replug USB. The drone boots normally with its settings intact.

## How to restore

### A. Exact restore from the flash image (preferred)

Puts back the stock firmware and every setting exactly as they were on 2026-10-18.

1. Enter the bootloader: `bl` in the CLI, or hold BOOT while plugging in USB. Close the Betaflight App.
2. `dfu-util -l` and confirm the `[0483:df11]` Internal Flash entry.
3. Write the image and reboot:
   ```bash
   dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D air65_stock_full.bin
   ```
   Add `sudo` if dfu-util reports a permission error.
4. Connect with the Betaflight App and check: `version` shows `e92c10887`, and `status` shows `BMI270` and `168MHz (PLLR-HSI)`.

The image only covers the flight controller. The ESCs and the ELRS receiver are untouched by any FC flash and need no restore.

### B. Stock firmware plus the settings backup (fallback)

Use this if the `.bin` is lost or damaged.

1. Get stock firmware for `BETAFPVG473_V2` in one of two ways:
   - The Betaflight App at master.app.betaflight.com, which builds development firmware in the cloud. Do not use an official release build; see the card above.
   - Build it from source:
     ```bash
     git worktree add ../bf-stock e92c10887
     cd ../bf-stock && make arm_sdk_install && make configs && make CONFIG=BETAFPVG473_V2
     ```
2. Flash it **without full chip erase** (see "Clock caveat").
3. Paste the `diff all` file into the CLI. It ends with `save`. Check the output for errors.
4. Verify `status` as in A.4.

### C. If the drone doesn't come back after any flash

The STM32's bootloader is in read-only ROM, so the board cannot be bricked by flashing. Hold BOOT while plugging in USB to force the bootloader, then do A.

### Clock caveat

The settings run the MCU from its internal oscillator (`system_hse_mhz = 0`), while the board config's default is an 8 MHz crystal. A **full chip erase** wipes the setting, so the first boot after it would try the crystal before the settings can be pasted back. It is not known whether this board has a working crystal or what the firmware does if it doesn't. So flash **without full chip erase**: Betaflight keeps saved settings across an update (it resets only setting groups whose layout changed), then paste the `diff all` on top anyway. After any flash, check that `status` still shows `PLLR-HSI`. If USB never comes back, use C.

## Why this backup exists: bank mode firmware

The owner develops a new Betaflight flight mode, **bank**, in a fork (its-wobble/betaflight, branch `banked-mode-init`; repo checkout `/workspaces/betaflight` in the dev container). Its design and terms are in that repo's `project_plan.md`. In bank mode the sticks become controls in the world frame:
- the throttle stick is **Speed**: the lower half is a speed, with the bottom braking to a stop; the upper half is a forward force
- the pitch stick is **Lift**, a vertical force: bottom 0 g, centre 1 g, top full thrust
- the yaw stick is **Bank**
- the roll stick is the **head check**, which turns the nose without changing the track

It has been flown only in a simulator (WebFPVSimulator, a 5-inch model at 1.62 g) until now. The open points for real hardware are in `project_plan.md`, section "Real hardware".

Facts for putting it on this drone:

- **Building:** the mode is compiled in only for MCUs with more than 512 KB of flash (`TARGET_FLASH_SIZE > 512` in `src/main/target/common_post.h`). This 512 KB G473 needs it forced on:
  ```bash
  make CONFIG=BETAFPVG473_V2 OPTIONS="USE_BANK_MODE"
  ```
  A trial build on 2026-10-18 used 79% of flash and 68% of RAM. Output: `obj/betaflight_<version>_STM32G474_BETAFPVG473_V2.hex`.
- **Flashing:** Betaflight App, "Load Firmware [Local]", full chip erase **off**. Then paste the `diff all` and check `status`.
- **Mode:** BANK has permanent mode ID 59. The plan is to put it on AUX2 high in place of acro, so AUX2 low (ANGLE) is the bail-out: `aux 5 59 1 1700 2100 0 0`.
- **Settings:**
  - `bank_thrust_to_weight` (x100, default 570, for the sim's 5-inch) must be measured for this drone. Hover in Angle with blackbox on and take c = (`rcCommand[3]` − 1000) / 1000 averaged over the hover; the setting is 100 / c^1.5.
  - `bank_mid_speed` (x100 g of drag, default 50): lower it (e.g. 10) for the first flights.
  - `set debug_mode = BANK`. Set `blackbox_sample_rate` low enough for the 2 MB flash.
- **Safety facts found in the code:**
  - In BANK, motor power comes from the Lift stick: at the centre it is hover power, armed or not. The arming check reads only the throttle (Speed) stick (`core.c`, `calculateThrottleStatus()`), so arming with BANK on and Lift centred spins straight up to hover power. So arm, take off and land in ANGLE, and only use BANK in the air.
  - In BANK the throttle stick is Speed. Switch BANK on or off only with the throttle stick near the hover position: leaving BANK with the stick at the bottom gives zero throttle in ANGLE.
  - Airmode is on by default (not changed in this diff), so the PIDs keep stabilising at zero Lift.
  - The BANK hooks for throttle-low, auto-disarm, runaway takeoff and failsafe have never run in the simulator. Bench-test them with props off before flying.
- **First-flight order:**
  1. Bench, props off.
  2. Switch in and out at a hover.
  3. Lift only.
  4. Speed (lower half, then braking).
  5. Gentle Bank and turns.
  6. Head checks and `bank_look_turn`.

  Record blackbox and DVR every flight.
- **Unknowns on real hardware:**
  - The speed estimate and the slip "ball" rely on rotor drag measured by the accelerometer, and ducts change that drag.
  - 1S battery sag moves the centre of Lift away from 1 g during a pack.
  - Whoop vibration adds noise to the accelerometer.

## Hardware test workflow, in detail

### Where things run

- **Dev container** (`/workspaces/betaflight`, which survives a container rebuild): builds the firmware, runs the unit tests and the simulator. It has **no USB access**.
- **Owner's computer** (CachyOS): Betaflight App, `dfu-util`, flashing and log downloads.
- **Moving the hex across:** download `obj/betaflight_<version>_STM32G474_BETAFPVG473_V2.hex` from the VS Code explorer (right-click, Download).
- **Before building for the drone,** run the bank checks as for any bank change: `make test_bank_unittest`, then the sim check (`bank-sim-loop` skill). Note the exact commit, and any uncommitted changes, that each hex was built from.

### After flashing the bank firmware

1. **Flash without full chip erase**, then paste the stock `diff all` (see "Clock caveat").
2. **Paste the bank settings:**
   ```
   aux 5 59 1 1700 2100 0 0
   set debug_mode = BANK
   set blackbox_sample_rate = 1/16
   set bank_mid_speed = 10
   set runaway_takeoff_prevention = OFF
   save
   ```
   Add `set bank_thrust_to_weight = <measured>` once it has been measured (next section).

   **Reminder: runaway takeoff.** Turn it off (above) until the firmware is fixed. In BANK it can disarm the drone in mid-air. The check switches itself off once the throttle *stick* has been above 20 % (with a stick deflected) for 0.5 s, or above 40 %. In BANK that stick is Speed, not the motor power (`core.c`, `calculateThrottlePercentAbs()`). After a launch in BANK, with Speed in the lower half, the check stays on all flight. It then disarms whenever a PID sum passes 60 % for 75 ms while the craft is rotating: a hard Bank entry, a hard brake or a touch on a wall. Under BANK, airmode's start latch (`airmode_start_throttle_percent = 25`) reads the same stick. The fix is to use the collective as the throttle percent under BANK in both places (open in `project_plan.md`, "Real hardware"). Once the fix is in, set it back `ON`.
3. **Check:**
   - `version` shows the fork's commit, not `e92c10887`.
   - `status` still shows `BMI270` and `168MHz (PLLR-HSI)`.
   - `get bank` lists the four settings: `bank_thrust_to_weight`, `bank_head_check_range`, `bank_look_turn` and `bank_mid_speed`.
   - The Modes tab shows BANK on AUX2 high.

About the logging rate: bank mode updates once per RX frame, about 250 Hz with this ELRS link, so logging at 1/16 (200 Hz from the 3.2 kHz loop) captures it while stretching the 2 MB flash. How many minutes that holds is an estimate; check it on the first flight. Use 1/4 or higher only for a gyro or PID question.

### Measuring `bank_thrust_to_weight`

1. Fly ANGLE (not BANK) on a fresh pack with blackbox logging.
2. Hover steadily for 10 to 15 s, after the first few seconds.
3. Average `rcCommand[3]` over the hover, and take c = (average − 1000) / 1000.
4. `bank_thrust_to_weight` = round(100 / c^1.5). For example, c = 0.35 gives 483.

Why this works:
- Outside BANK, the mixer's throttle is `rcCommand[THROTTLE] − 1000` plus the throttle angle correction (`thr_corr_value`, default 0 and unchanged here).
- In BANK, the collective × 1000 replaces exactly that term (`mixer.c`).
- So the hover throttle in ANGLE is the collective BANK needs for 1 g, and the mode's model is thrust = T/W × collective^1.5.
- `thr_mid`/`thr_expo` are at their defaults (50/0), so the throttle is linear. The Speed stick reads the same `rcCommand[THROTTLE]`, so a throttle curve would reshape Speed too.

Telling whether the value is right, once flying BANK:
- **Set too low:** the drone climbs at centre Lift right after take-off.
- **Set too high:** it sinks, and the thrust trim (`debug[2]`) settles below 1000.

1S sag lowers the real T/W through the pack. `vbat_sag_compensation` is the candidate fix, listed in `project_plan.md`.

### Bench test, props off (expected results from the code; verify each)

Use a battery, with props off, and the radio bound.

| Do | Expect |
|---|---|
| Flick AUX2 through its positions | Modes tab and OSD show ANGLE, HORIZON, BANK |
| Sensors tab, drone level and still | Accelerometer about 1 g on Z, about 0 on X and Y |
| Arm in ANGLE, throttle low, flick to BANK with the right stick centred | Motors rise to about hover power: Lift centre is 1 g |
| In BANK, pull the right stick fully back | Motors drop to idle (0 g Lift) and keep stabilising (airmode is on) |
| In BANK, move the throttle (Speed) stick | Motor outputs shift as the mode asks for forward tilt; the drone can't tilt on the bench, so watch the Motors tab or a log |
| Disarm, then arm with BANK already on, the throttle stick at the bottom and the right stick centred | Arms, and motors go straight to hover power; this is why arming happens in ANGLE |
| Armed in BANK, switch the radio off | Failsafe drops BANK (it yields to failsafe) and runs the normal failsafe |

The throttle-low, auto-disarm, runaway takeoff and failsafe "landed" hooks under BANK have never run in the simulator, so watch for any surprise disarm or any failure to disarm.

### Logs

- **Getting them off:** Betaflight App, Blackbox tab, "Save flash to file", then "Erase flash" before the next flight.
- **Decoding:** `blackbox_decode` (betaflight/blackbox-tools) writes one CSV per armed session. The Blackbox Explorer views them.
- **The `DEBUG_BANK` fields:**

| Field | Meaning | Unit |
|---|---|---|
| `debug[0]` | Speed estimate | 0.001 g of drag |
| `debug[1]` | Target speed | 0.001 g of drag |
| `debug[2]` | Thrust trim | 0.001 (1000 = model right) |
| `debug[3]` | Forward force command | 0.001 g |
| `debug[4]` | Thrust to weight in use | 0.01 |
| `debug[5]` | Turn feedforward | 0.1 deg/s |
| `debug[6]` | Ball (slip) | 0.001 g |
| `debug[7]` | Vertical force command minus 1 g | 0.001 g |

- **Sim tools:** the simulator's `npm run bank:log` and `--replay` read the sim's own CSV logs only, not blackbox. A converter (blackbox CSV to the sim log format) is needed before hardware flights can be summarised or replayed that way. As of 2026-10-18 it does not exist.

### Per-flight record

Note with each log:
- date
- firmware commit, and whether there were uncommitted changes
- `bank_*` settings
- pack and its resting voltage
- indoor or outdoor, and the wind
- what was flown, in the order of the first-flight list
- anything that felt wrong, with the time into the flight
