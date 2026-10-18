# Hardware notes (Air65)

Owner decisions about flying bank mode on real hardware that aren't in [air65-backup/reference.md](../air65-backup/reference.md) or project_plan.md's "Real hardware". Read reference.md first for the drone, restore, build, bench test and log fields.

## Take-off in BANK from the ground (2026-10-18)

- **Take-off:** the owner will probably take the Air65 off in BANK from the ground, not in ANGLE or acro as reference.md's procedure says. So ground take-off behaviour matters on hardware, including arming with BANK on: centre Lift spins up to the collective for 1 g.
- **T/W:** keep `bank_thrust_to_weight` at the high end (5.70) for both crafts, while the measuring thrust trim is in. The trim fully corrects a value set too high, but only partly corrects one set too low, because it leaks back to 1. In the sim the whoop (true value about 4.8) took off from the ground in BANK fine with the setting up to 2.5 times too high.
- **Without the trim** (the parked trial in [snapshots.md](snapshots.md)), the rule flips: the Air65 would need its measured T/W, and its own inflow slope.
- **Why:** the take-off is where a wrong T/W shows, because the trim can't measure on the ground.
- **How to apply:** check any T/W, arming or Lift change against a ground take-off in BANK on the whoop as well as the five inch.
