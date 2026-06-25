# franka_pose_estimation

A lean ROS 2 wrapper around the [FoundationPose](https://github.com/NVlabs/FoundationPose)
live pipeline. It runs RealSense capture + FoundationPose `register`/`track_one`
in a background thread and serves the latest 6D object pose — in **camera optical
coordinates** — through a service.

```
srv/GetObjectPose
  string object_name              # request (reserved; ignored for now)
  ---
  bool valid                      # true once registered & tracking
  geometry_msgs/PoseStamped pose  # object pose in camera_frame, meters
  string message                  # status string
```

`pose` is FoundationPose's raw `ob_in_cam` (the mesh-origin frame expressed in the
camera optical frame), unchanged from `live_pose.py`.

## Why it runs inside the FoundationPose container

FoundationPose needs its GPU stack (torch / nvdiffrast / pytorch3d / pyrealsense2),
which lives only in the FP image. The **`live`** image is built on
`foundationpose:blackwell` → **Ubuntu 22.04 + system python 3.10**, which is exactly
what ROS 2 Humble targets. So Humble's `rclpy` can be apt-installed into the *same*
interpreter that runs FoundationPose — one process, no bridge. The container already
runs `--network host --ipc=host`, so a matching `ROS_DOMAIN_ID` is all that's needed
to join the franka DDS graph.

> Note: this only works for the `live` / `blackwell` image. The legacy
> `foundationpose:latest` (Ubuntu 20.04 + conda py3.8) cannot take apt Humble.

## One-time: add ROS 2 + this package to the FP `live` image

Append to `docker/Dockerfile.live` (or run interactively in the container):

```dockerfile
# --- ROS 2 Humble (matches Ubuntu 22.04 / py3.10) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release software-properties-common && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base ros-humble-geometry-msgs \
        python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*
```

`rclpy` is then importable by the system `python3` that already has FoundationPose.

## Build (inside the container)

Mount the multipanda workspace and build just this package:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/multipanda_ros2          # mounted into the container
colcon build --packages-select franka_pose_estimation
source install/setup.bash
```

Build the same package on the host/franka side too, so clients have the
`franka_pose_estimation/srv/GetObjectPose` type.

## Run

```bash
export ROS_DOMAIN_ID=0               # must match the franka graph
ros2 launch franka_pose_estimation pose_service.launch.py \
     prompt:=nut mesh_file:=demo_data/nut/mesh/nut.obj
```

Query the pose (from anywhere on the same domain):

```bash
ros2 service call /foundationpose_pose_service/get_object_pose \
     franka_pose_estimation/srv/GetObjectPose "{}"
```

Until the object is detected and registered, `valid` is `false` and `message`
reports the current state (`looking for '<prompt>'`, `register`, `tracking`, …).

## Key parameters

| param | default | meaning |
|-------|---------|---------|
| `foundationpose_dir` | `/workspace` (or `$FOUNDATIONPOSE_DIR`) | FP repo root, added to `sys.path` |
| `mesh_file` | `demo_data/nut/mesh/nut.obj` | CAD mesh (abs, or relative to `foundationpose_dir`) |
| `prompt` | `nut` | Grounding-DINO text prompt for first-frame detection |
| `camera_frame` | `camera_color_optical_frame` | `header.frame_id` of the published pose |
| `zfar` | `1.0` | ignore depth beyond this (m) |
| `est_refine_iter` / `track_refine_iter` | `5` / `2` | FoundationPose refine iterations |
