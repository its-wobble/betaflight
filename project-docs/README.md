# Project docs (bank mode)

Notes about the bank mode work that need to outlast a session and a container rebuild. Claude's own memory lives under `/home/devpod` and is lost on a rebuild, so it goes here instead. This folder is fork-only: drop it before an upstream PR, as CLAUDE.md's Conventions say.

What goes where:
- [project_plan.md](../project_plan.md): the spec, the decisions and the design history.
- This folder: what doesn't fit there. That means where the snapshots are, the owner's decisions about hardware, and the archived session plans.
- [air65-backup/](../air65-backup/): the test drone's backup, build, flashing and log reference.

## Contents

- [snapshots.md](snapshots.md): revert points and parked experiments under `.bank-snapshots/`, and what each one holds.
- [hardware-notes.md](hardware-notes.md): owner decisions for flying the Air65 in BANK.
- [discontinued/](discontinued/): ideas the owner dropped, kept for the reasoning. So far that's the take-off thrust calibration, replaced by a manual trim the user sets.
- [plans/](plans/): plans from past sessions, by date. They are a record of what was intended, not the current state; project_plan.md says what actually happened.

## Keeping it current

One topic per section, dated. Say why a note matters and how to use it. Update or delete a note when it stops being true, rather than adding a contradicting one. Don't repeat what the code, git history or project_plan.md already record.
