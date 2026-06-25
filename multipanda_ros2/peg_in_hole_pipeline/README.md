# peg_in_hole_pipeline

State-machine pipeline that detects a peg + hole with FoundationPose, grasps the peg, and
inserts it — built on the running MoveIt `/move_group` and the `franka_gripper` actions.

## Components (one responsibility each)
| file | role |
|------|------|
| `perception.py` | calls the FoundationPose `/foundationpose/<obj>/get_pose` services, transforms poses into `panda_link0` via TF |
| `grasp.py`      | grasp + insertion pose generation (hardcoded fixed transforms, all offsets are parameters) |
| `motion.py`     | MoveIt motion: pose goals, joint goals (HOME), straight-line Cartesian insertion — no `moveit_py` dependency |
| `gripper.py`    | `franka_msgs/action/Grasp` (with **force**), `Move`, `Homing` |
| `states.py`     | the state machine: `HOME → DETECT_OBJECTS → COMPUTE_GRASP → MOVE_TO_GRASP → CLOSE_GRIPPER → MOVE_TO_HOLE → INSERT → FINISHED`, with `ERROR` on any failure |
| `pipeline_node.py` | wires it together; all tunables are ROS2 parameters |

## Prerequisites (multipanda container, `ROS_DOMAIN_ID=1`)
1. **MoveIt with the gripper** (required — the pipeline uses the `panda_manipulator` group / `panda_hand_tcp` tip, which only exist when the hand is loaded):
   ```
   ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2 load_gripper:=true use_rviz:=true
   ```
2. **Camera** (aligned depth): `ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true`
3. **FoundationPose** (other container) publishing `/foundationpose/<obj>/get_pose`.

## Build & run
```
cd ~/multipanda_ws
colcon build --packages-select peg_in_hole_pipeline
source install/setup.bash
ros2 run peg_in_hole_pipeline pipeline                       # or the launch file
ros2 run peg_in_hole_pipeline pipeline --ros-args -p grasp_force:=30.0 -p vel_scale:=0.05
```

## Tune before running on hardware
- `grasp_offset_xyz`, `grasp_width`, `grasp_orientation_xyzw` — where/how to grab the peg.
- `home_joints` — defaults to the Franka "ready" pose.
- `hole_approach_height`, `insertion_depth` — the insertion geometry.
- `vel_scale`/`acc_scale` — start low (0.05) with a hand on the e-stop.

## TODOs (in code)
visual servoing / multi-sample detection (`perception.py`), adaptive grasp from peg
orientation (`grasp.py`), force/impedance-controlled insertion (`motion.py`), and a real
behavior tree once the flow branches (`states.py`).
