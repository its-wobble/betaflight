# Bank mode: take-off thrust calibration (design spec, discontinued)

> **Discontinued (owner, 2026-10-18).** Not implemented, and not to be implemented. The owner would rather have the user configure this by hand, with a trim, than have the firmware measure the T/W in an automatic take-off. That manual trim is not designed yet. The spec is kept as a record of the idea and of what was worked out for it: ground effect on the whoop, why the ground can't measure the T/W, and the lift-off detection. See also the parked no-trim trial in [../snapshots.md](../snapshots.md).

Spec agreed with the owner on 2026-10-18, for implementation in a later session. Read [CLAUDE.md](../../CLAUDE.md) and the "Terms" section of [project_plan.md](../../project_plan.md) first. This spec uses those terms (Lift, Speed, Bank, thrust, collective, thrust trim, T/W).

## Goal

Each flight, measure the craft's real thrust to weight during a short automatic take-off. Then **lock** it for the rest of the flight, so the collective no longer follows the live thrust trim.

Why: at a fixed collective, props make less thrust when air comes down through them (a climb) and more when it comes up through them (a sink). Left alone, that slows any vertical motion and brings the craft to rest relative to the air. Today the thrust trim measures that change and cancels it (it keeps the delivered Lift at what the stick asks), so a climb or sink at centre stick carries on. With the T/W locked, the trim no longer corrects the collective, and the props' own damping returns. The owner accepts that this is relative to the air, not the ground, and that it brings some side effects (see "Accepted consequences").

The trim can't be locked at arming. On the ground the accelerometer reads 1 g whatever the props do, so nothing can be measured there. That is why `bank_thrust_to_weight` is kept deliberately high (5.70), and the trim corrects it in the air. The measurement has to happen in the air, just after lift-off, and clear of ground effect.

## Owner's decisions

1. **Trigger:** after arming in BANK the craft stays inactive (motors at idle). Calibration starts automatically when the pilot raises the **Lift stick above neutral**.
2. **Sticks disabled** during calibration: Bank, Speed and the head check are ignored until it finishes.
3. **The firmware ramps the thrust itself.** While the Lift stick stays above neutral, it raises the thrust steadily from zero until the accelerometer detects lift-off. How far the stick is above neutral doesn't matter.

   A pilot-led ramp (the stick sets the thrust, with a capped rate of rise) was considered and dropped. A slowly raised stick that is then held can float a whoop in its ground-effect cushion with the vertical force at 1 g, so lift-off is never detected. A rising automatic ramp always climbs out.
4. **Abort:** Lift stick below neutral at any point stops the process and puts the motors at idle.
5. **A short self-hold just above the ground**, to get clear of ground effect before the value is measured and set.
6. **The locked value holds until disarm.**
7. **Out of scope for now:** failsafe, crash flip (turtle) mode, runaway takeoff prevention.
8. **Battery sag is accepted** as the pack fades. Betaflight's existing `vbat_sag_compensation` already scales bank mode's collective ([mixer.c:266](../../src/main/flight/mixer.c#L266)). It is a setting to try, not part of this work.

## Defaults chosen in this spec (confirm with the owner if in doubt)

- **Lift stick in its centre deadband during the ramp:** pause. Hold the current collective without ramping further. The Lift stick is spring-centred, so letting go pauses rather than aborts. Above neutral resumes the ramp; below neutral aborts.
- **During the self-hold** the Lift stick is ignored, except that below neutral still aborts (rule 4). An abort there drops the craft from up to about 0.3 m.
- **Starting a ramp:** the Lift stick must have been at or below neutral since arming, the same way `reverseReady` gates the reverse zone. Arming with the Lift stick already raised must not start a ramp.
- **Ramp cap:** if the cap is reached with no lift-off (snagged, upside down, too heavy), abort to idle.
- **Handover after the lock:**
  - Lift is held at 1 g until the Lift stick has been back in its centre deadband once. Otherwise a pilot still holding the stick up would get a sudden climb.
  - Bank and Speed use the existing `tiltLocked` rule: no tilt until both sticks have been at zero once.
- **If the self-hold can't produce a valid measurement** (timeout, saturation, value out of range): don't lock. Enable the sticks and fly as today (setting × live trim) until disarm.
- **Engaging BANK while already airborne** (armed in another mode): no calibration. Use a lock left from earlier in this arming if there is one, otherwise fly as today.

## Phases

One state machine in `flight/bank_mode.c`, reset on disarm. Suggested names: `IDLE`, `RAMP`, `CLIMB_OUT`, `SETTLE`, `LOCKED`, plus `UNCALIBRATED` (fly as today). Calibration only runs if BANK was active at the moment of arming.

### IDLE (armed, on the ground)

- **Outputs:** collective 0 (motors at idle), angle targets 0, yaw rate 0. With collective 0, `bankModeIsThrottleLow()` is already true, so `core.c` holds the I-term off.
- **Ground bias:** while at rest, average the vertical force (the same projection the trim uses in `bankCompute`). It should be exactly 1 g. Keep `bias = average − 1` for the velocity integration below.
- **Exit:** Lift stick above neutral (subject to the arming gate above) → `RAMP`.

### RAMP (on the ground, thrust rising)

- **Ramp in modelled thrust** from 0, through the configured T/W: `collective = bankCollectiveFromThrust(t, setting)`, with `t` rising at about **0.5 g/s**. The real thrust isn't known until measured. The setting is kept at or above the true value, so the real thrust rises no faster than the model says.
- **Cap:** about **3 g modelled**, which covers a true T/W down to about 1.9 at a 5.70 setting. At the cap with no lift-off → abort to `IDLE`.
- **Attitude:** level, yaw rate 0, Bank, Speed and head check ignored.
- **Hold the I-term off until lift-off**, so tilted ground can't wind it up and tip the craft as it lifts. The precedent is `pidSetItermReset(launchWingHoldsIterm())` in [core.c:868](../../src/main/fc/core.c#L868): stabilisation runs and the I-term stays reset. Under BANK, `throttleActive` is true once the collective is above 0, so a bank-mode equivalent of that hook is needed.
- **Keep updating the ground bias** while no lift-off has been seen.
- **Lift stick in the deadband:** pause. **Below neutral:** abort to `IDLE` (collective 0).
- **Lift-off detection:** the vertical force, PT1-filtered at about 20 Hz to reject spool-up vibration, above **1.05 g for 50 ms** → `CLIMB_OUT`. On the ground the force can't exceed 1 g, so a reading above it means the craft is accelerating upward.

### CLIMB_OUT (airborne, leaving ground effect)

Ground effect is large on a whoop. In the sim's model (`plant.c`, Cheeseman–Bennett form with the duct doubling the effective radius), the props make about +33 % thrust parked on the ground, +7 % one prop diameter up and +2 % at two. A value locked at the moment of lift-off would leave the whoop about 25 % short in free air.

- **Vertical velocity:** integrate `(vertical force − 1 − bias)` from the start of the lift-off detection window, in g·s. It starts from rest, and the bias was taken at rest on the same attitude, so it stays accurate over the 2–3 s this phase and `SETTLE` take.
- **Collective:** Lift = `1 + Kv · (vClimb − v)`, clamped to about 0.5–1.5 g. Collective from that Lift with the level-attitude tilt compensation, through a T/W seeded at lift-off as `measured force / modelled thrust` at the ramp's collective. That keeps the collective continuous at the switch.
- **Starting values to tune:**
  - `vClimb` ≈ **0.03 g·s**. That is about 0.3 m/s at Earth's g. The sim flies at 1.62 g, so the same value is about 0.48 m/s there.
  - `Kv` ≈ **3 /s**.
  - About **1 s** of climb, roughly 0.3 m: several prop diameters on the whoop, about two on the five inch.
- **Attitude** level, the I-term released, the other sticks still ignored. Below neutral on the Lift stick aborts.
- **Exit:** after the climb time → `SETTLE`.

### SETTLE (airborne, measuring)

- Same loop with a target of 0: Lift = `1 − Kv · v`.
- After about **0.5 s** to settle, measure over about **0.5 s**: T/W = mean of `vertical force / (lagged modelled thrust per T/W × tiltCos)`. Lag the model as the trim does (`BANK_TRIM_LAG_HZ`).
- Use a sample only when |v| is small, the mixer isn't saturated (`motorsSaturated`) and the tilt is within the trim's limits. Extend the window up to a timeout of about 2 s; on timeout → `UNCALIBRATED`.
- **Range check:** reject a result outside about 1.2–10 → `UNCALIBRATED`.
- **Success:** store the value and go to `LOCKED`.

### LOCKED (normal flight until disarm)

- **The collective uses the locked T/W with no trim:** `bankCollectiveFromThrust(..., lockedThrustToWeight)`.
- **The thrust trim keeps measuring** against the locked base and still feeds:
  - the limits that use `trimmedThrustToWeight` today (the Lift clamp and the tilt budget);
  - the drag scaling of the Speed estimate.

  In short: `trimmedThrustToWeight = locked × trim` everywhere it's used today, except the collective, which uses `locked` alone.
- **Lift** is held at 1 g until the Lift stick has been in its centre deadband once, then follows the stick as today. Bank and Speed follow the `tiltLocked` rule.
- **Store the locked value outside `bankState`.** `bankModeUpdate()` zeroes `bankState` whenever BANK is off, and the lock must survive switching BANK off and on until disarm. Clear it on disarm.

## Outputs and logging

- **No new debug fields or getters** (CLAUDE.md: firmware changes are for flight behaviour only).
- **`bankModeGetThrustToWeight()`** should return the T/W in use: the locked value once set, otherwise the setting. The sim already logs it (`bf_bank.c`, `out[5]`), so the lock shows in flight logs with no new outputs.
- **`DEBUG_BANK[4]`** ("Thrust To Weight") should report the same value. The phase can be inferred from the collective and the sim's plant state.

## Accepted consequences

With the collective no longer following the trim:

- **Centre stick:** a climb or sink slows to zero relative to the air, within about 1–2 s and more slowly in a bank. This is the goal.
- **Fast forward flight:** the craft sinks steadily, because the thrust lost to forward airflow is no longer added back. The steady sink is about the thrust shortfall × RPM × prop pitch.
- **Braking** (tilting back into the airflow): the craft balloons upward.
- **Banks:** a gentle climb or sink, depending on how far the `collective^1.5` thrust curve is off at a bank's higher collective.
- **Updrafts and prop wash:** the craft rides them.
- **Off centre stick:** Lift is still a force command, but the delivered force is no longer exact.
- **Battery sag:** the craft sinks more as the pack fades (see decision 8).

If the fast-flight sink proves too much, the next step is to scale the locked value by a model of forward-flight thrust loss: the trim's existing `inflow` term, Speed × forward tilt. Not part of this work.

## Testing

**Unit tests** (`src/test/unit/bank_unittest.cc`, `make test_bank_unittest`):
- The ramp:
  - it rises at the set rate whatever the stick height above neutral;
  - centring the stick pauses it and raising the stick again resumes it;
  - below neutral aborts.
- The arming gate (Lift stick raised at arming).
- Abort at the cap.
- Lift-off detection: fires at the threshold and duration, and ignores short spikes.
- The ground bias and the velocity integration.
- `CLIMB_OUT` and `SETTLE` against a synthetic plant with a true T/W of 4.8, a setting of 5.70 and a decaying ground-effect gain: the lock lands near 4.8.
- Fallback to `UNCALIBRATED`.
- The lock survives a BANK toggle and clears on disarm.
- Engaging BANK in the air skips calibration.
- Lift is held at 1 g until the stick is centred.
- After the lock, the collective ignores the trim while the limits still use it.

**Sim** (`bank-sim-loop` skill):
- **Ground take-offs on both crafts.** Compare the locked T/W with the truth: about 4.8 on the whoop, which has ground effect in the sim, and 5.70 on the five inch, which has none. Check that no ground take-off goes wrong.
- **After the lock:**
  - vertical drift over 20–30 s at centre stick;
  - how fast a climb stops after centring;
  - height change in fast flight, braking and held banks.
- **Expect to update the sim's sorties.** `scripts/bank-check.js` (around lines 157–167) takes off with `lift: 0.65` for 0.8 s and then centres; it needs the new take-off timing, and possibly new thresholds. `bank-log.js --replay` of logs flown before this change will differ from the take-off on.
- **Then ask the owner** to fly it by hand (`npm run serve`, `?fw=bank`) and read their log.

**Hardware:** later, following [air65-backup/reference.md](../../air65-backup/reference.md).

## After implementing

- **project_plan.md:**
  - add a section for this feature;
  - amend "Lift stays a force": the stick is still a force, but with the T/W locked the props' damping now slows vertical motion;
  - amend the Terms entry for the thrust trim: after the lock it measures but no longer sets the collective.
- **No new CLI setting:** use constants, as the rewrite does. If one is added, it goes in `pg/bank_mode.h` and `cli/settings.c` with a PG version bump.
- **Fork-only files:** this spec file is fork-only. Add it to CLAUDE.md's list, or fold it into project_plan.md once the feature is done.
