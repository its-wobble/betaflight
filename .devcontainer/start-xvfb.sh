#!/usr/bin/env bash
# Start a virtual X display for headless Chromium (the WebFPVSimulator's
# browser checks: lint:shell, lint:input, shots.js). Debian's Chromium builds
# ANGLE's Vulkan path for X11 only, so even --headless needs an X server to
# get WebGL; without one every check times out waiting for the shell.
# Idempotent: does nothing if the display is already up.
set -euo pipefail
DISPLAY_NUM="${DISPLAY:-:99}"
if pgrep -f "Xvfb $DISPLAY_NUM" >/dev/null; then
  exit 0
fi
nohup setsid Xvfb "$DISPLAY_NUM" -screen 0 1920x1080x24 -nolisten tcp >/tmp/xvfb.log 2>&1 &
