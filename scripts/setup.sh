#!/usr/bin/env bash
#
# One-shot setup so every clone has the same workspace.
# Run once from the repo root after cloning:  ./scripts/setup.sh
#
# What it does:
#   1. Pulls the pinned multipanda_ros2 driver (git submodule).
#   2. Links our packages (src/insertion, src/ball_balance) into the
#      multipanda_ros2 colcon source tree, so `colcon build` finds them
#      next to franka_bringup & co.
#   3. Marks those links as ignored inside the submodule so it stays clean.
#
# Build itself happens inside the multipanda Docker container (see README).

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRIVER_DIR="$REPO_ROOT/multipanda_ros2"

echo "==> [1/3] Fetching pinned driver (submodule, humble branch)…"
git -C "$REPO_ROOT" submodule update --init --recursive

if [ ! -d "$DRIVER_DIR" ]; then
  echo "ERROR: $DRIVER_DIR not found after submodule init." >&2
  exit 1
fi

echo "==> [2/3] Linking our packages into the driver source tree…"
for pkg in insertion ball_balance; do
  target="$DRIVER_DIR/$pkg"
  src="../src/$pkg"   # relative to DRIVER_DIR
  if [ -L "$target" ] || [ -e "$target" ]; then
    echo "    - $pkg already linked, skipping"
  else
    ln -s "$src" "$target"
    echo "    - linked $pkg -> $src"
  fi
done

echo "==> [3/3] Keeping the submodule working tree clean…"
# For a submodule, the real git dir lives under <super>/.git/modules/...,
# so resolve it instead of assuming a literal .git/ folder.
GIT_DIR="$(git -C "$DRIVER_DIR" rev-parse --absolute-git-dir)"
EXCLUDE_FILE="$GIT_DIR/info/exclude"
mkdir -p "$GIT_DIR/info"
for pkg in insertion ball_balance; do
  grep -qxF "/$pkg" "$EXCLUDE_FILE" 2>/dev/null || echo "/$pkg" >> "$EXCLUDE_FILE"
done
echo "    - marked links ignored in $EXCLUDE_FILE"

cat <<'EOF'

Setup done. Next steps (inside the driver dir, see README for details):

    cd multipanda_ros2
    ./tools/setup_env      # builds the Docker image (first time only, slow)
    ./run                  # drops you into the container, ws at ~/multipanda_ws
    colcon build
    source install/setup.bash

Sanity check (simulation, no real arm needed):

    ros2 launch franka_bringup franka_sim.launch.py
EOF
