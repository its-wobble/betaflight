# Flashing the latest bank firmware to the Air65 R

Steps for putting the newest `banked-mode-init` build on the drone. The background (the board, the backups, how to restore stock) is in [reference.md](reference.md).

The firmware is built in the dev container, which has no USB, and flashed from the owner's computer with the Betaflight App.

## 1. Build (dev container)

```bash
cd /workspaces/betaflight
git log -1 --oneline                        # note the commit; `git status` for uncommitted changes
make test_bank_unittest                     # bank unit tests
cd $SIM_DIR && npm run bank:cycle           # simulator check
cd /workspaces/betaflight
make configs                                # first time only, or after a config submodule update
make CONFIG=BETAFPVG473_V2 OPTIONS="USE_BANK_MODE"
```

- Bank mode is left out of 512 KB chips by default (`TARGET_FLASH_SIZE > 512` in `common_post.h`), so `OPTIONS="USE_BANK_MODE"` is required.
- The output is `obj/betaflight_<version>_STM32G474_BETAFPVG473_V2.hex`.

## 2. Copy the hex to your computer

In the VS Code explorer, right-click the `.hex` under `obj/` and choose **Download**.

## 3. Back up the drone's current settings

Connect in the Betaflight App, open the CLI, run `diff all` and save the output to a file. Keep it with the other backups in this folder.

## 4. Flash

1. Betaflight App, **Firmware Flasher** tab.
2. **Load Firmware [Local]** and pick the `.hex`.
3. Full Chip Erase can be on or off. The board's crystal works (see reference.md, "Clock caveat"), so it boots either way.
4. **Flash Firmware**, and wait for it to reboot and reconnect.

If it does not reconnect: hold the BOOT button while plugging in USB, then flash again. The STM32 bootloader is in ROM, so the board cannot be bricked by a flash.

## 5. Restore the settings

Expect the settings to be reset: Full Chip Erase wipes them, and Betaflight also resets them when the config layout changes between builds.

Open the CLI and paste [restore-bank-firmware.txt](restore-bank-firmware.txt). It is the stock tune and switches plus the bank settings, and it ends with `save`. If you changed settings on the drone since that file was made, paste the `diff all` from step 3 instead, or update the file first.

Read the CLI output for errors. A renamed or removed `bank_*` setting shows up there.

The file includes:

```
aux 4 59 2 1700 2100 0 0              # BANK on AUX3 high (ANGLE is AUX3 middle, the bail-out)
set debug_mode = BANK
set blackbox_sample_rate = 1/16
set bank_mid_speed = 50                # 5 m/s at the centre of the Speed stick, for indoors
set runaway_takeoff_prevention = OFF
```

**Keep `runaway_takeoff_prevention = OFF`** until the fix is in (`project_plan.md`, "Real hardware"). In BANK it reads the Speed stick, not the motor power, and can disarm the drone in mid-air.

`set system_hse_mhz = 0` in the file runs the MCU from its internal oscillator, as BetaFPV shipped it. Deleting that line before pasting keeps it on the 8 MHz crystal. Both run at 168 MHz.

## 6. Check

In the CLI:

- `version` shows the commit noted in step 1.
- `status` shows `BMI270`, `168MHz`, and `PLLR-HSI` (or `PLLR-HSE` if you left out `system_hse_mhz = 0`).
- `get bank` lists `bank_mid_speed` and `bank_corner_rate` (since 2026-10-19 the drone's thrust to weight and drag rate are learned in flight, not set).

In the App:

- The Modes tab shows BANK on AUX3 high.
- The Sensors tab, with the drone level and still, shows about 1 g on Z.

## 7. Bench test, props off

Run the bench table in reference.md ("Bench test, props off") before flying. Remember that arming with BANK on and the right (Lift) stick centred spins the motors straight to hover power.

## 8. Record

Note with every flight log: the commit (and whether there were uncommitted changes), the `bank_*` settings, and the rest of reference.md's "Per-flight record".
