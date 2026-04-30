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

# `set -u` would be nice for catching typos, but ROS setup scripts
# (/opt/ros/*/setup.bash and the workspace's local_setup.bash) reference
# vars like AMENT_TRACE_SETUP_FILES that aren't always defined — sourcing
# them with -u kills the deploy. Use -e and -o pipefail only.
set -eo pipefail

REPO=${REPO:-$HOME/stable_bot_repo}
WS=${WS:-$HOME/ros2_ws}
ROS_DISTRO=${ROS_DISTRO:-kilted}

cd "$REPO"

echo "==> [1/8] check local changes"
# Most local diffs on the Pi are just GUI-saved gain YAMLs, IVA
# alignment YAMLs, or new tuning_data/ bag dirs — auto-commit those
# so `git pull --rebase` doesn't die on dirty-tree. Anything else
# (Python edits, manual config tweaks) is reported and the deploy
# stops so the user can resolve it explicitly.
LOCAL_CHANGES=$(git status --porcelain || true)
if [ -n "$LOCAL_CHANGES" ]; then
    # Files in the porcelain output that are NOT yaml and NOT under
    # tuning_data/. awk index 2 is the path; works for the standard
    # ' M file' / '?? file' formats. Repo has no spaces in filenames
    # so this is safe.
    UNSAFE=$(echo "$LOCAL_CHANGES" \
              | awk '{print $2}' \
              | grep -vE '\.(yaml|yml)$|^tuning_data/' || true)
    if [ -n "$UNSAFE" ]; then
        echo "  ! cannot auto-commit. Non-yaml/non-tuning_data changes:"
        echo "$UNSAFE" | sed 's/^/    /'
        echo
        echo "  Resolve manually before re-running:"
        echo "    git checkout -- <file>             # discard"
        echo "    git stash                          # set aside"
        echo "    git add <file> && git commit -m 'msg'   # keep"
        exit 1
    fi
    echo "  auto-committing yaml + tuning_data:"
    echo "$LOCAL_CHANGES" | sed 's/^/    /'
    git add -A -- '*.yaml' '*.yml' tuning_data/ 2>/dev/null || true
    if ! git diff --cached --quiet; then
        git commit -m "Local: yaml/tuning_data snapshot (pi_deploy.sh)" >/dev/null
        echo "  ✓ committed local snapshot"
    else
        echo "  (nothing actually staged — skipping commit)"
    fi
else
    echo "  no local changes"
fi

echo "==> [2/8] git pull --rebase"
git pull --rebase

echo "==> [3/8] git push (auto-committed snapshot ↑)"
# Push so any local snapshot we just made lands on origin too. If
# there's nothing to push or push fails (e.g., no creds), keep going
# — the live deploy doesn't depend on origin being current.
git push 2>&1 | sed 's/^/  /' || echo "  (push skipped/failed — push manually if needed)"

echo "==> [4/8] copy systemd unit files + daemon-reload"
sudo cp "$REPO/stewart_bringup/scripts/stable_bot.service" \
        "$REPO/stewart_bringup/scripts/stable_bot_gui.service" \
        /etc/systemd/system/
sudo systemctl daemon-reload

echo "==> [5/8] source ROS overlay"
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "$WS/install/local_setup.bash" ]; then
    # shellcheck source=/dev/null
    source "$WS/install/local_setup.bash"
fi

echo "==> [6/8] colcon build (--symlink-install)"
cd "$WS"
colcon build --symlink-install \
  --packages-select stewart_vision stewart_bringup jugglebot_interfaces

echo "==> [7/8] restart services"
sudo systemctl restart stable_bot_gui.service
sudo systemctl restart stable_bot.service

echo "==> [8/8] verify (5 s soak)"
sleep 5
EXPECTED=$(git -C "$REPO" rev-parse --short HEAD | tr -d '[:space:]')
echo "  expected git sha: $EXPECTED"
# Pull the last 500 lines once and inspect — running journalctl twice
# sometimes raced when the journal hadn't flushed yet, and grep -q in
# a pipeline interacts badly with pipefail in some bash versions.
JOURNAL=$(journalctl -u stable_bot.service -n 500 --no-pager 2>/dev/null || true)
echo "  recent oak_driver banner / config:"
echo "$JOURNAL" | grep -E '\[boot\]|Building OAK|Depth subsystem' | tail -5 || true
BOOT_LINE=$(echo "$JOURNAL" | grep '\[boot\] oak_driver' | tail -1 || true)
if [ -n "$BOOT_LINE" ] && [[ "$BOOT_LINE" == *"code sha=$EXPECTED"* ]]; then
    echo "  ✓ live code SHA matches HEAD ($EXPECTED)"
elif [ -n "$BOOT_LINE" ]; then
    echo "  ! SHA mismatch — running code is older than HEAD."
    echo "    expected: $EXPECTED"
    echo "    line    : $BOOT_LINE"
else
    echo "  ! no [boot] banner found in last 500 lines — service may still be starting up."
    echo "    Re-run:  journalctl -u stable_bot.service -n 500 --no-pager | grep '\[boot\]'"
fi
