#!/usr/bin/env bash
# Clone the WebFPVSimulator next to this repo and prepare it for the bank mode
# loop (see .claude/skills/bank-sim-loop). Idempotent: an existing checkout is
# left on whatever branch it is on.
set -euo pipefail

SIM_DIR="${SIM_DIR:-$HOME/WebFPVSimulator}"
SIM_REPO="${SIM_REPO:-https://github.com/its-wobble/WebFPVSimulator.git}"
SIM_BRANCH="${SIM_BRANCH:-banked-mode-sim-2}"
BF_DIR="$(cd "$(dirname "$0")/.." && pwd)"

if [ ! -d "$SIM_DIR/.git" ]; then
  git clone --branch "$SIM_BRANCH" "$SIM_REPO" "$SIM_DIR"
fi

# The sim pins Betaflight 4.5.1 as a submodule. That commit is in this repo's
# object store (upstream history), so borrow it instead of downloading.
git -C "$SIM_DIR" submodule update --init --reference "$BF_DIR" vendor/betaflight

if [ -f "$SIM_DIR/package-lock.json" ]; then
  (cd "$SIM_DIR" && npm ci --no-audit --no-fund)
fi
