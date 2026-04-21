#!/usr/bin/env bash
# Stop all Stable-Bot processes in WSL. Called by stop_stable_bot.bat
# but also runnable directly: `bash _kill_all.sh`.
echo "killing stewart_control_node..."
pkill -9 -f stewart_control_node
echo "killing rosbridge..."
pkill -9 -f rosbridge
echo "killing xsens drivers..."
pkill -9 -f xsens_mti_node
echo "killing rosapi..."
pkill -9 -f rosapi
echo "killing gui_server..."
pkill -9 -f gui_server.py
echo "killing ros2 launch..."
pkill -9 -f "ros2 launch stewart"
echo "freeing port 9090 (rosbridge)..."
fuser -k 9090/tcp 2>/dev/null || true
echo "freeing port 8080 (gui_server)..."
fuser -k 8080/tcp 2>/dev/null || true
echo "done."
