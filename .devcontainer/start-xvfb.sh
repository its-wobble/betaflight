#!/usr/bin/env bash
# Start a virtual X display for headless Chromium (the WebFPVSimulator's
# browser checks: lint:shell, lint:input, shots.js). Debian's Chromium builds
# ANGLE's Vulkan path for X11 only, so even --headless needs an X server to
# get WebGL; without one every check times out waiting for the shell.
# Idempotent: does nothing if the display is already up. Called on demand by
# chromium-xvfb.sh (SIM_CHROME_BIN), or by hand.
set -euo pipefail
DISPLAY_NUM="${DISPLAY:-:99}"
SOCKET="/tmp/.X11-unix/X${DISPLAY_NUM#:}"
if ! pgrep -f "^Xvfb $DISPLAY_NUM" >/dev/null; then
  # Xvfb only creates the socket dir when run as root
  mkdir -p /tmp/.X11-unix && chmod 1777 /tmp/.X11-unix 2>/dev/null || true
  nohup setsid Xvfb "$DISPLAY_NUM" -screen 0 1920x1080x24 -nolisten tcp >/tmp/xvfb.log 2>&1 &
fi
for _ in $(seq 50); do
  [ -S "$SOCKET" ] && pgrep -f "^Xvfb $DISPLAY_NUM" >/dev/null && exit 0
  sleep 0.1
done
echo "start-xvfb: display $DISPLAY_NUM not up after 5 s, see /tmp/xvfb.log" >&2
exit 1
