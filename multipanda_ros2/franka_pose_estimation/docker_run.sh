#!/bin/bash
# Host-side launcher: start the FoundationPose container with GPU + RealSense +
# both repos mounted, then run the pose service inside it (installs ROS 2 Humble
# on first run, builds the package, launches). Pass extra launch args through, e.g.
#   bash docker_run.sh prompt:=driller mesh_file:=demo_data/driller/mesh/driller.obj
set -e

FP_DIR=${FOUNDATIONPOSE_DIR:-/home/minga-09/FoundationPose}
WS_DIR=$(cd "$(dirname "$0")/.." && pwd)          # multipanda_ros2 root
IMAGE=${FP_IMAGE:-foundationpose:live}            # built from docker/Dockerfile.live
DOMAIN=${ROS_DOMAIN_ID:-0}

echo "[docker_run] FP repo : $FP_DIR  (-> /workspace)"
echo "[docker_run] ws repo : $WS_DIR"
echo "[docker_run] image   : $IMAGE   ROS_DOMAIN_ID=$DOMAIN"

xhost +local:root >/dev/null 2>&1 || true
docker rm -f fp_pose_service 2>/dev/null || true
docker run --rm -it \
  --gpus all --privileged -v /dev:/dev \
  --network host --ipc=host \
  --env NVIDIA_DISABLE_REQUIRE=1 \
  -e ROS_DOMAIN_ID="$DOMAIN" \
  -e PYOPENGL_PLATFORM=egl -e HF_HOME=/workspace/.hf_cache \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$FP_DIR":/workspace -w /workspace \
  -v "$WS_DIR":"$WS_DIR" \
  --name fp_pose_service \
  "$IMAGE" \
  bash "$WS_DIR/franka_pose_estimation/container_setup.sh" "$@"
