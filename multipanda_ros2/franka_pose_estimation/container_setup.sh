#!/bin/bash
# In-container bootstrap: ensure ROS 2 Humble, build franka_pose_estimation, run it.
# Invoked by docker_run.sh, but you can also run it by hand inside the container.
# Build artifacts go to /tmp/fp_ws so the host workspace's build/ is never touched.
set -e

WS_DIR=$(cd "$(dirname "$0")/.." && pwd)          # multipanda_ros2 root
BUILD_BASE=/tmp/fp_ws/build
INSTALL_BASE=/tmp/fp_ws/install

# 1. ROS 2 Humble (only if missing) ----------------------------------------
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[setup] installing ROS 2 Humble (one-time) ..."
  export DEBIAN_FRONTEND=noninteractive
  apt-get update
  apt-get install -y --no-install-recommends curl gnupg lsb-release software-properties-common
  add-apt-repository -y universe
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list
  apt-get update
  apt-get install -y --no-install-recommends \
    ros-humble-ros-base ros-humble-geometry-msgs \
    ros-humble-rosidl-default-generators \
    python3-colcon-common-extensions
fi
source /opt/ros/humble/setup.bash

# 2. build just our package ------------------------------------------------
echo "[setup] colcon build franka_pose_estimation ..."
cd "$WS_DIR"
colcon build --packages-select franka_pose_estimation \
  --build-base "$BUILD_BASE" --install-base "$INSTALL_BASE"
source "$INSTALL_BASE/setup.bash"

# 3. sanity: confirm rclpy + FoundationPose coexist in this interpreter -----
python3 - <<'PY'
import importlib, sys
sys.path.insert(0, "/workspace")
for m in ("rclpy", "estimater", "pyrealsense2", "franka_pose_estimation.srv"):
    importlib.import_module(m)
    print(f"[check] import {m}: OK")
PY

# 4. run -------------------------------------------------------------------
echo "[setup] launching pose service ..."
exec ros2 launch franka_pose_estimation pose_service.launch.py "$@"
