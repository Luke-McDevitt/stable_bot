#!/usr/bin/env bash
# Helper script called by start_stable_bot.bat.
# Brings up can0, then starts gui_server.py in the foreground.
# Do not call directly — use the .bat (or invoke `bash ...` yourself).
set -e

# Bring can0 up. Try NOPASSWD sudo first; fall back to interactive sudo
# (user will type password in the cmd window).
if sudo -n /usr/sbin/ip link show can0 2>/dev/null | grep -q "state UP"; then
    echo "[_launch_gui_server] can0 already up"
else
    echo "[_launch_gui_server] bringing up can0..."
    sudo -n /usr/sbin/ip link set can0 up type can bitrate 1000000 2>/dev/null \
        || sudo /usr/sbin/ip link set can0 up type can bitrate 1000000 \
        || echo "[_launch_gui_server] WARNING: couldn't bring up can0 — CAN won't work"
    sudo /usr/sbin/ip link set can0 txqueuelen 1000 2>/dev/null || true
fi

echo ""
echo "[_launch_gui_server] starting gui_server.py on :8080"
echo ""

exec python3 "$HOME/ros2_ws/src/stewart_bringup/scripts/gui_server.py"
