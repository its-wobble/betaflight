# Project plan: "bank" flight mode

A new flight mode, **"bank"**. Development conventions (target independence, compile-time gating, tests, PR rules) are in [AGENTS.md](AGENTS.md).

Goal: fly a drone like a car on a banked track. The three control axes are coupled down to two, with a spare stick axis for look-around. Mode 2 stick mapping:

| Stick | Channel | Function |
|---|---|---|
| Left U/D | throttle | **vertical force/acceleration command**, not raw throttle. Centre = thrust balances weight (hover / no vertical acceleration), using a thrust-to-weight ratio (estimated or user-set); up/down adds/removes net vertical force |
| Left L/R | yaw | **head check**: yaw the nose away from the direction of travel *without* changing the direction of travel; springs back to neutral when the stick centres |
| Right U/D | pitch | **forward/backward force command**, coupled pitch + collective: pitching the craft forward also raises collective thrust so the horizontal force component grows while the vertical component is unchanged |
| Right L/R | roll | **bank** left/right = turn in the horizontal plane |

Design decisions so far:
- **Attitude only** (gyro/ACC); no GPS or baro required.
- Keep an internal virtual *travel heading* (a course reference, similar in spirit to headfree). Nose yaw = travel heading + head-check offset; the head-check offset must not affect the velocity direction.
- **Force-vector control (decoupled axes)**: the sticks command a net force vector, and the mixer of attitude + collective realises it. Required collective for tilt angle θ (pitch) and φ (bank) is T = (m·g + F_z) / (cos θ · cos φ), so with left U/D at neutral, moving right U/D from centre to forward pitches the craft and increases throttle by 1/cos θ, giving forward force without changing vertical force. Left U/D adds the vertical term F_z. Collective is therefore always computed from the commanded tilt, never taken directly from the stick.
- **Coordinated banked turn**: the roll stick commands bank angle φ; the mode generates the matching yaw rate so that tan φ = v·ω / g, and applies the same 1/cos tilt compensation so the vertical force stays balanced — the forces should model a "perfect bank".

Open questions (decide with the user before implementing): source of thrust-to-weight (CLI setting vs estimate); mapping from right U/D to forward force / tilt angle (and reverse behaviour), and from left U/D to vertical force (scaling, deadband around neutral), thrust saturation limits when tilt compensation demands more than max throttle; max bank angle and head-check range / return rate; interaction with arming, failsafe and ANGLE/HORIZON/HEADFREE/ALT_HOLD/POS_HOLD priority.

Likely touchpoints (use `pos_hold_multirotor.c`, `alt_hold_multirotor.c` and `poshold_unittest.cc` / `althold_unittest.cc` as the closest patterns):
- `fc/rc_modes.h` (new `BOXBANK` before `BOXID_FLIGHTMODE_LAST`), `fc/runtime_config.h` (new `BANK_MODE` bit + map entry), `msp/msp_box.c` (box name, new unique `permanentId`, availability in the box-enable list)
- `fc/core.c` (mode enable/disable, cf. `HEADFREE_MODE`), `fc/rc.c` (setpoint shaping, cf. the `HEADFREE_MODE` block), `flight/imu.c` (cf. headfree attitude handling), `flight/pid.c`, `flight/mixer.c` (thrust compensation)
- new `flight/bank_*.c/h` with a PG config (`pg/pg_ids.h`, `cli/settings.c`), gated by a new `USE_BANK_MODE` define in `target/common_post.h`
- mode indicators: `osd/osd_elements.c`, `osd/osd_warnings.c`, `io/ledstrip.c`, `telemetry/*`
- `src/test/unit/bank_unittest.cc` plus `bank_unittest_SRC` in `src/test/Makefile`
