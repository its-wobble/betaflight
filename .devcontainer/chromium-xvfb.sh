#!/usr/bin/env bash
# Chromium for the WebFPVSimulator's browser checks (SIM_CHROME_BIN). Starts
# the Xvfb display first if it is not running, so the checks work however the
# container was started.
set -euo pipefail
export DISPLAY="${DISPLAY:-:99}"
bash "$(dirname "$0")/start-xvfb.sh"
exec /usr/bin/chromium "$@"
