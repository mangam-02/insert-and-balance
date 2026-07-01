# Peg-in-Hole Insertion on a Franka Panda — Vision + Force

> **A 6D-pose-driven, force-regulated peg-in-hole pipeline for the Franka Emika Panda. Detect, grasp, and compliantly insert — with a spiral search when the hole hides.**

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-blue?logo=python&logoColor=white)
![MoveIt](https://img.shields.io/badge/MoveIt-2-0A7BBB)
![Robot](https://img.shields.io/badge/Robot-Franka%20Emika%20Panda-ffcf00)
![Perception](https://img.shields.io/badge/Perception-FoundationPose-76B900?logo=nvidia&logoColor=white)
![Status](https://img.shields.io/badge/Status-Hackathon%20Prototype-yellow)

---

## Demo

<table>
  <tr>
    <td align="center"><b>Detect → grasp → force-regulated insertion on the Franka Panda</b></td>
  </tr>
  <tr>
    <td><img src="demo.gif" width="640"/></td>
  </tr>
</table>

---

## 🏆 Built at the Europe Embodied Hackathon 2026

This project — by **Team `manipulatoren`** — was built in **48 hours** at the
**[Europe Embodied Hackathon](https://europe-embodied.com/events/hackathon)**
(Munich, **June 24–26, 2026**), organized by **RoboTUM & START Munich** and billed
as *"Europe's largest physical AI playground."* Teams tackled real challenges
sourced from robotics labs and industry partners, on real hardware, judged on live
demos.

We took the **Intel — Industrial Robotics Arm Challenge (Challenge 1: Insertion /
peg-in-hole)** on a Franka Emika Panda. Everything in this repository was written
during the event, on the arm, against the shared hackathon workstations.

| | |
|---|---|
| **Event** | Europe Embodied Hackathon — 48 h, Munich, 24–26 June 2026 |
| **Challenge** | Intel Industrial Robotics Arm — peg-in-hole insertion with 3D-printed shapes |
| **Robot** | Franka Emika Panda + Franka Hand gripper |
| **Partners** | Franka Robotics · Intel · AWS · ESRA |
| **Our approach** | **Hybrid** — learned 6D pose (FoundationPose) + classical compliant insertion |

> The challenge brief we worked from is preserved in [CHALLENGE.md](CHALLENGE.md).

---

## Overview

Peg-in-hole is the canonical *contact-rich* manipulation task: the last few
millimetres are unforgiving of pose error, and a rigid position controller either
misses the hole or jams the peg. The challenge brief offered three routes —
**classical vision + force**, a **learned VLA / imitation policy**, or a **hybrid**.
We chose the hybrid, because it plays to the arm's real-time impedance controller
while keeping perception general:

- **Coarse alignment from vision.** A neural 6D pose estimator (NVIDIA
  **FoundationPose**) locates the peg *and* the socket from a wrist-mounted
  RealSense — no fiducials, just the CAD meshes.
- **Fine insertion from force.** The last centimetre runs under **Cartesian
  impedance control**: soft along the insertion axis, force-regulated to a gentle
  contact, with an **Archimedean spiral search** that finds the hole when the peg
  lands on the surface instead of dropping in.

The whole thing is a **single, sequential ROS 2 node** you can read top to bottom —
no hidden state machine — with every geometric and force parameter exposed as a ROS
parameter for fast tuning on the robot.

---

## Method

### Pipeline

```
        RealSense (wrist)                     Franka Panda + Hand
               |                                      |
               v                                      |
   +-----------------------+                          |
   |   FoundationPose       |  6D pose (peg, insert)  |
   |   register + track     |  in camera frame        |
   +-----------------------+                          |
               |  hand-eye extrinsics (TF)            |
               v                                      v
   +-------------------------------------------------------------+
   |                  peg_in_hole_pipeline (one node)            |
   |                                                             |
   |  HOME ─▶ DETECT ─▶ [TIP-UP if peg is lying down]            |
   |       ─▶ COMPUTE GRASP ─▶ MOVE TO GRASP ─▶ CLOSE GRIPPER    |
   |       ─▶ LIFT ─▶ MOVE TO HOLE ─▶ INSERT ─▶ RETURN HOME      |
   |                                        |                    |
   |   free-space moves: MoveIt (Pilz PTP)  |  contact: Cartesian|
   |                                        |  impedance + force |
   |                                        |  regulation + spiral|
   +-------------------------------------------------------------+
```

Object poses are detected **before** any grasp pose, marker, or motion is computed,
so the arm never plans against a stale or empty detection.

### 1 · Perception — FoundationPose 6D pose

`franka_pose_estimation` is a lean ROS 2 wrapper around
[FoundationPose](https://github.com/NVlabs/FoundationPose). It runs RealSense
capture + FoundationPose `register`/`track_one` in a background thread inside the
GPU container and serves the latest 6D pose of each object (peg, insert) in the
**camera optical frame**. The pipeline node transforms those poses into the robot
base frame via the hand-eye calibration.

### 2 · Hand-eye calibration

`franka_camera_calibration` performs **eye-in-hand extrinsic calibration**: the
existing MoveIt stack drives the arm through a CSV of Cartesian poses, a fixed
ChArUco board is photographed at each, and `cv2.calibrateHandEye` solves for the
camera pose relative to the flange. A companion RealSense **intrinsics** calibration
lives at the repo root (`calibrate_intrinsics.py`, `charuco_board.png`).

### 3 · Grasp — top-down, with an experimental tip-up recovery

The grasp is a fixed top-down transform on the detected peg (all offsets are
parameters). Before grasping, the node **classifies the peg orientation**: it
rotates the peg's long axis into the base frame and measures its tilt from vertical.
If the peg is **lying on its side**, an experimental **tip-up maneuver** grasps it
tilted, lifts it clear of the table, rotates it upright, sets it back down, and
re-detects, aiming to recover a fallen peg instead of aborting.

> ⚠️ **Work in progress.** The tip-up recovery is implemented and steps through in
> RViz, but it was **not fully reliable by the end of the hackathon** — the tilted
> grasp and stand-up orientation still need tuning on the robot. The main grasp +
> force-regulated insertion path (upright peg) is the part that works.

### 4 · Insertion — force-regulated impedance + spiral search

The contact phase hands the arm from the position controller to a **Cartesian
impedance controller** (soft along Z, stiff laterally). A force loop servos the
equilibrium pose downward to regulate the contact force to a small target while
advancing to the insertion depth, with a hard `|F|` safety cap that aborts on
over-force.

If the peg presses on the surface without descending (stuck on the rim), a
**spiral search** engages: an outward **Archimedean spiral** in x/y is added to the
equilibrium pose under a gentle capped press, until the peg drops in — detected **by
position** (the EE descends to within tolerance of the target depth), not by force.
The controller is always handed back to the position controller before the node
retracts and returns home.

---

## Technical Features

- **Learned 6D pose** (FoundationPose) for peg *and* socket — fiducial-free, CAD-driven
- **Eye-in-hand hand-eye calibration** (ChArUco + `cv2.calibrateHandEye`) and RealSense intrinsics
- **Peg orientation classification** and an experimental **tip-up regrasp** for a fallen peg *(work in progress — not fully reliable yet)*
- **Cartesian-impedance force regulation** on the insertion axis with a hard over-force safety cap
- **Archimedean spiral search** that finds the hole on surface contact, ending by descent depth
- **MoveIt free-space motion** (Pilz PTP joint moves, Cartesian straight-line paths) with no `moveit_py` dependency
- **Optional impedance free-space moves** (speed-limited equilibrium ramp) as an alternative to MoveIt
- **Everything is a ROS 2 parameter** — grasp geometry, forces, stiffness, spiral, timeouts
- **Operator affordances**: RViz debug-pose markers, a latched `/peg_in_hole/state` topic for a frontend, and a `step_through` mode that pauses between phases
- **Identical sim/real stack** via `multipanda_ros2` (MuJoCo mirror of the hardware interface)

---

## Repository Structure

```
manipulatoren/
├── CHALLENGE.md                       # the Intel peg-in-hole challenge brief
├── calibrate_intrinsics.py            # RealSense intrinsics calibration (ChArUco)
├── generate_charuco_board.py          # printable ChArUco board generator
├── charuco_board.png                  # the printed calibration target
├── realsense_intrinsics.yaml          # calibrated camera intrinsics
│
└── multipanda_ros2/                   # ROS 2 driver (fork) + our packages
    ├── peg_in_hole_pipeline/          # ★ the insertion pipeline (single node)
    │   └── peg_in_hole_pipeline/pipeline_node.py
    ├── franka_pose_estimation/        # ★ FoundationPose 6D-pose ROS 2 service
    │   └── scripts/foundationpose_pose_service.py
    ├── franka_camera_calibration/     # ★ eye-in-hand hand-eye calibration
    ├── objects/                       # peg.stl, insert.stl (CAD meshes)
    └── ...                            # franka_bringup, moveit_config, gripper, ...
```

★ = written by Team `manipulatoren` at the hackathon.

---

## Usage

All commands run inside the `multipanda_ros2` dev container (`ROS_DOMAIN_ID=1`).
See each package's own README for the full detail
([peg_in_hole_pipeline](multipanda_ros2/peg_in_hole_pipeline/README.md),
[franka_pose_estimation](multipanda_ros2/franka_pose_estimation/README.md),
[franka_camera_calibration](multipanda_ros2/franka_camera_calibration/README.md)).

### 0 · Bring up the container

```bash
cd multipanda_ros2
./run                                                   # opens a shell in the container
docker exec -it --user developer multipanda-container bash   # extra terminals
```

### 1 · Robot stack + camera

```bash
# MoveIt with the gripper (required — the pipeline uses panda_manipulator / panda_hand_tcp)
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2 load_gripper:=true use_rviz:=true

# RealSense with aligned depth
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

### 2 · Perception (FoundationPose, GPU container)

```bash
cd franka_pose_estimation
bash docker_run.sh prompt:=peg mesh_file:=objects/peg.stl
# publishes /foundationpose/<obj>/pose for peg and insert
```

### 3 · Run the insertion pipeline

```bash
cd ~/multipanda_ws
colcon build --packages-select peg_in_hole_pipeline
source install/setup.bash

ros2 run peg_in_hole_pipeline pipeline
# tune on the fly:
ros2 run peg_in_hole_pipeline pipeline --ros-args \
     -p grasp_force:=30.0 -p vel_scale:=0.05 -p step_through:=true
```

Watch the computed grasp / hole / insertion poses as colored arrows in RViz, and
step through phase-by-phase with `step_through:=true` and a hand on the e-stop.

### Gripper (manual, for setup)

```bash
ros2 launch franka_gripper gripper.launch.py robot_ip:=172.16.0.2 arm_id:=panda
ros2 action send_goal /panda_gripper/homing franka_msgs/action/Homing "{}"
ros2 action send_goal /panda_gripper/move   franka_msgs/action/Move "{width: 0.08, speed: 0.1}"
```

---

## Implementation Details

| Component | Choice | Rationale |
|---|---|---|
| Object pose | FoundationPose (6D, model-based) | Fiducial-free; generalizes across the 3D-printed shapes |
| Camera | Wrist-mounted Intel RealSense, aligned depth | Eye-in-hand keeps the socket in view during approach |
| Extrinsics | ChArUco + `cv2.calibrateHandEye` (park) | Insertion is unforgiving of camera→base pose error |
| Free-space motion | MoveIt / Pilz **PTP** | Planned, limit-aware, no steady-state error for large moves |
| Insertion | Cartesian impedance, soft Z stiffness | Compliant contact; gentle regulated force into the hole |
| Hole search | Archimedean spiral, ends by descent depth | Recovers when the peg lands on the surface, not the hole |
| Fallen peg | Orientation classification + tip-up regrasp *(WIP)* | Aims to stand a lying peg upright before grasping — not yet fully reliable |
| Safety | Hard `|F|` cap + `error_recovery` | Reflexes stay loose enough not to trip mid-insertion |
| Structure | One sequential node, all params | Readable top-to-bottom; fast to tune on hardware |

---

## Acknowledgements

Franka Panda driver and MuJoCo sim mirror from
[`tenfoldpaper/multipanda_ros2`](https://github.com/tenfoldpaper/multipanda_ros2)
(branch `humble`). 6D pose from
[NVlabs/FoundationPose](https://github.com/NVlabs/FoundationPose). Challenge,
hardware, and workstations provided by **Intel**, **Franka Robotics**, and the
**Europe Embodied Hackathon** organizers (RoboTUM & START Munich).

---

*Built in 48 hours at the **Europe Embodied Hackathon 2026**, Munich — Team `manipulatoren`.*
