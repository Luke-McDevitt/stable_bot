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
# Most local diffs on the Pi are just GUI-saved artifacts: gain YAMLs,
# IVA alignment YAMLs, new tuning_data/ bag dirs, or V0 NN weight
# tunings (v0_weights.json + the rebuilt v0_*.blob/.onnx pair). All
# of those are operator-tuned data, not source edits, so we auto-
# commit them rather than block on a dirty tree. Anything else
# (Python edits, manual config tweaks) gets reported and the deploy
# stops so the user can resolve it explicitly.
#
# The allowlist regex below is the single source of truth — keep it
# in sync with the operator's tuning surface as new GUI-writable
# artifacts get added.
SAFE_RE='\.(yaml|yml)$|^tuning_data/|^stewart_vision/blobs/'
# Hard deny-list: paths that must NEVER be auto-committed even if
# they match SAFE_RE. Auto-tune session bag/*.mcap files are tens-to-
# hundreds of MB; one snuck through on 2026-05-01 (073749Z, 113.66 MB)
# and blocked every push for the next 3 commits because the file
# exceeded GitHub's 100 MB limit. .gitignore alone is not enough — a
# tracked file (e.g., committed before its gitignore rule landed)
# sticks around until explicitly untracked, and `git add <dir>`
# happily re-stages it.
SAFE_DENY_RE='/bag/|\.mcap$|metadata\.yaml$'
# Untrack any bag/ contents that may already be in the index from
# pre-gitignore commits. Files stay on disk; just removed from the
# index so the next `git add` won't re-include them.
git ls-files 'tuning_data/auto_tune_*/bag/*' 2>/dev/null \
  | xargs -r git rm --cached --quiet 2>/dev/null || true

LOCAL_CHANGES=$(git status --porcelain || true)
if [ -n "$LOCAL_CHANGES" ]; then
    # Files in the porcelain output that don't match the allowlist.
    # awk index 2 is the path; works for the standard ' M file' /
    # '?? file' formats. Repo has no spaces in filenames so this is safe.
    UNSAFE=$(echo "$LOCAL_CHANGES" \
              | awk '{print $2}' \
              | grep -vE "$SAFE_RE" || true)
    if [ -n "$UNSAFE" ]; then
        echo "  ! cannot auto-commit. Source-tree changes detected:"
        echo "$UNSAFE" | sed 's/^/    /'
        echo
        echo "  Resolve manually before re-running:"
        echo "    git checkout -- <file>             # discard"
        echo "    git stash                          # set aside"
        echo "    git add <file> && git commit -m 'msg'   # keep"
        exit 1
    fi
    echo "  auto-committing operator-tuned artifacts:"
    echo "$LOCAL_CHANGES" | sed 's/^/    /'
    # Stage by explicit paths from `git status --porcelain` rather than
    # pathspec globs — `git add '*.yaml'` doesn't always match
    # recursively from the repo root depending on git version, and
    # silently no-ops which lands us right back at "cannot pull with
    # unstaged changes". Loop over the porcelain output and add each
    # safe path explicitly.
    while IFS= read -r path; do
        [ -n "$path" ] && git add -- "$path"
    done < <(echo "$LOCAL_CHANGES" \
              | awk '{print $2}' \
              | grep -E "$SAFE_RE")
    # Belt-and-suspenders: even though .gitignore + the deny-list
    # above should keep bag/ out, double-check the staged set and
    # un-stage anything that snuck through. If we let a >100 MB
    # mcap into the commit it blocks every future push until
    # someone runs git filter-branch on the Pi.
    BAD_STAGED=$(git diff --cached --name-only | grep -E "$SAFE_DENY_RE" || true)
    if [ -n "$BAD_STAGED" ]; then
        echo "  ! deny-list match — un-staging large-file paths:"
        echo "$BAD_STAGED" | sed 's/^/    /'
        while IFS= read -r p; do
            [ -n "$p" ] && git reset HEAD -- "$p" >/dev/null 2>&1 || true
        done < <(echo "$BAD_STAGED")
    fi
    if ! git diff --cached --quiet; then
        git commit -m "Local: operator-tuned snapshot (pi_deploy.sh)" >/dev/null
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
