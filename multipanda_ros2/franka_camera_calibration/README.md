# franka_camera_calibration

Eye-in-hand extrinsic calibration for a Franka Panda wrist camera (e.g. Intel
RealSense). The robot is driven through a CSV of Cartesian poses using the
**existing MoveIt stack**; at each pose a ChArUco board (fixed in the world) is
photographed and detected; after the last pose, OpenCV hand-eye calibration
computes the **camera pose relative to the gripper/flange frame**.

```
CSV poses ──▶ MoveGroup (/move_group) ──▶ settle ──▶ grab image ──▶ detect ChArUco
                                                                          │
                                       after last pose ◀── cv2.calibrateHandEye ◀── + TF base→gripper
```

## Contents

| Component | What it does |
|-----------|--------------|
| `generate_charuco_board` | Generates a printable ChArUco board (PNG + PDF at true scale) |
| `calibrate_wrist_camera` | The calibration node (move → capture → detect → solve) |
| `config/calibration.yaml` | All node parameters (topics, frames, board, motion) |
| `config/charuco_board.yaml` | Board params (keep in sync with the printout) |
| `config/poses_template.csv` | Example Cartesian poses CSV |
| `launch/calibration.launch.py` | Launches the calibration node |

## 1. Build

```bash
cd ~/multipanda_ws            # the colcon workspace (inside the dev container)
colcon build --packages-select franka_camera_calibration
source install/setup.bash
```

## 2. Print the ChArUco board

```bash
ros2 run franka_camera_calibration generate_charuco_board \
    --squares-x 5 --squares-y 7 \
    --square-length 0.04 --marker-length 0.03 \
    --dictionary DICT_5X5_1000 \
    --output-dir ~/wrist_cam_calibration
```

This writes `charuco_board.{png,pdf,yaml}`. **Print the PDF at 100% / "actual
size"** (no fit-to-page). Then **measure a black square with calipers** and put
the measured `square_length` / `marker_length` into
`config/charuco_board.yaml` *and* the `charuco:` block of
`config/calibration.yaml`. Calibration accuracy depends directly on these
numbers. Mount the printed board flat and rigid; keep it fixed during the run.

## 3. Start the robot stack + camera

```bash
# Terminal A — MoveIt (already working position control)
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2 use_rviz:=true

# Terminal B — RealSense driver (your existing launch)
ros2 launch realsense2_camera rs_launch.py
```

Confirm the camera topic names and set them in `config/calibration.yaml`
(`image_topic`, `camera_info_topic`):

```bash
ros2 topic list | grep -i camera
```

The intrinsics are read from `camera_info`. If your camera does not publish
`camera_info`, set `camera_info_yaml` to a standard OpenCV camera-calibration
YAML instead.

## 4. Prepare the poses CSV

Columns: `name, x, y, z, qx, qy, qz, qw` — the pose of `panda_hand_tcp` in
`panda_link0` (position in metres, orientation as a quaternion). See
`config/poses_template.csv`.

Guidelines for a good hand-eye result:
- **8–15 poses**, all keeping the board comfortably in view.
- Vary **both position and orientation** — especially include **tilts/rotations**
  about different axes. Pure translations make the problem ill-conditioned.

The template values are placeholders — replace them with poses valid for *your*
board placement. The easiest way to author them: jog the robot in RViz to good
viewpoints and read the current `panda_hand_tcp` pose, e.g.

```bash
ros2 run tf2_ros tf2_echo panda_link0 panda_hand_tcp
```

## 5. Run the calibration

```bash
ros2 launch franka_camera_calibration calibration.launch.py \
    poses_csv:=/home/developer/wrist_cam_calibration/poses.csv
```

The node moves to each pose, captures an annotated detection image
(`pose_XXX_*.png`) into `output_dir`, and after the last pose prints and saves:

```
~/wrist_cam_calibration/wrist_camera_extrinsics.yaml
```

containing the camera optical frame expressed in `gripper_frame`
(`panda_hand` by default), plus a ready-to-use `static_transform_publisher`
command. A small `consistency_residual` (AX=XB) indicates a good result; a
large one usually means too few/parallel poses or wrong board dimensions.

## Key parameters (`config/calibration.yaml`)

| Param | Default | Notes |
|-------|---------|-------|
| `poses_csv` | *(required)* | Absolute path to the poses CSV |
| `image_topic` | `/camera/camera/color/image_raw` | Verify with `ros2 topic list` |
| `camera_info_topic` | `/camera/camera/color/camera_info` | Source of intrinsics |
| `planning_group` | `panda_manipulator` | Plans `panda_link0`→`panda_hand_tcp` |
| `base_frame` | `panda_link0` | Planning frame for the CSV poses |
| `pose_link` | `panda_hand_tcp` | Link the CSV poses are expressed for |
| `gripper_frame` | `panda_hand` | Rigid camera mount frame; result is camera pose **in here** |
| `settle_time` | `2.0` | Seconds to wait after each move before capturing |
| `vel_scale` / `acc_scale` | `0.1` | MoveIt velocity/accel scaling (keep slow) |
| `handeye_method` | `park` | `tsai`/`park`/`horaud`/`andreff`/`daniilidis` |
| `charuco.*` | 5×7, DICT_5X5_1000 | **Must match the printed board** |

## Notes

- The MoveGroup client is self-contained (depends only on `moveit_msgs`), so it
  needs no `pymoveit2` / `moveit_py` — it just talks to the running
  `/move_group` action server.
- Works with OpenCV 4.6 (old `aruco` API) and 4.7+ (new `CharucoDetector` API).
- `gripper_frame` should be the link the camera is **rigidly bolted to**. The
  result is the transform from that frame to the camera optical frame. Set it to
  whatever your camera mount is fixed to (e.g. `panda_hand` or `panda_link8`).
