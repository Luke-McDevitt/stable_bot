#!/usr/bin/env bash
# Stable-Bot Pi deploy: pull → systemd refresh → colcon build → restart
# both services, then verify the running git SHA.
#
# Usage:  ~/stable_bot_repo/stewart_bringup/scripts/pi_deploy.sh
# (or symlink it to ~/bin/deploy-stable-bot for shorter typing)
#
# Why this exists: every individual command we forget — copying the
# .service files, daemon-reload, sourcing the overlay before colcon,
# restarting BOTH services — has cost a 4-hour debug round at least
# once. Bake the recipe in.

set -euo pipefail

REPO=${REPO:-$HOME/stable_bot_repo}
WS=${WS:-$HOME/ros2_ws}
ROS_DISTRO=${ROS_DISTRO:-kilted}

cd "$REPO"

echo "==> [1/6] git pull --rebase"
git pull --rebase

echo "==> [2/6] copy systemd unit files + daemon-reload"
sudo cp "$REPO/stewart_bringup/scripts/stable_bot.service" \
        "$REPO/stewart_bringup/scripts/stable_bot_gui.service" \
        /etc/systemd/system/
sudo systemctl daemon-reload

echo "==> [3/6] source ROS overlay"
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "$WS/install/local_setup.bash" ]; then
    # shellcheck source=/dev/null
    source "$WS/install/local_setup.bash"
fi

echo "==> [4/6] colcon build (--symlink-install)"
cd "$WS"
colcon build --symlink-install \
  --packages-select stewart_vision stewart_bringup jugglebot_interfaces

echo "==> [5/6] restart services"
sudo systemctl restart stable_bot_gui.service
sudo systemctl restart stable_bot.service

echo "==> [6/6] verify (5 s soak)"
sleep 5
EXPECTED=$(git -C "$REPO" rev-parse --short HEAD)
echo "  expected git sha: $EXPECTED"
echo "  recent oak_driver banner / config:"
journalctl -u stable_bot.service -n 200 --no-pager \
  | grep -E '\[boot\]|Building OAK|Depth subsystem' | tail -5 || true

if journalctl -u stable_bot.service -n 200 --no-pager \
        | grep -q "code sha=$EXPECTED"; then
    echo "  ✓ live code SHA matches HEAD ($EXPECTED)"
else
    echo "  ! could not confirm SHA — service may still be starting up."
    echo "    Re-run:  journalctl -u stable_bot.service -n 200 --no-pager | grep '\[boot\]'"
fi
