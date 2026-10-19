# Hardware notes (Air65)

Owner decisions about flying bank mode on real hardware that aren't in [air65-backup/reference.md](../air65-backup/reference.md) or project_plan.md's "Real hardware". Read reference.md first for the drone, restore, build, bench test and log fields.

## Take-off in BANK from the ground (2026-10-18)

- **Take-off:** the owner will probably take the Air65 off in BANK from the ground, not in ANGLE or acro as reference.md's procedure says. So ground take-off behaviour matters on hardware, including arming with BANK on: centre Lift spins up to the collective for 1 g.
- **T/W:** since 2026-10-19 there is no setting: the thrust model is the fixed `BANK_THRUST_TO_WEIGHT` (5.70), and the thrust trim corrects it in flight. It stays high on purpose: the trim fully corrects a value too high, but only partly one too low, because it leaks back to 1. In the sim the whoop (true value about 4.9) flies the whole sortie on it. A model at 3.0 slipped the five inch's turn 14°, and one at 9.0 surged the whoop's first cruise ("Learned airframe" in project_plan.md). A learner for it was tried and dropped the same day: it changed none of those results.
- **Without the trim** (the parked trial in [snapshots.md](snapshots.md)), the rule flips: the Air65 would need its measured T/W, and its own inflow slope.
- **Why:** the take-off is where a wrong T/W shows, because the trim can't measure on the ground.
- **How to apply:** check any T/W, arming or Lift change against a ground take-off in BANK on the whoop as well as the five inch.
