# insert-and-balance

Intel Industrial Robotics Arm Challenge — Franka Emika Panda.
Two tasks: **Challenge 1** peg-in-hole insertion, **Challenge 2** balancing a
table-tennis ball on a TCP-mounted plate. Full brief in [CHALLENGE.md](CHALLENGE.md).

## Repo layout
```
insert-and-balance/
├── CHALLENGE.md            # the full challenge brief
├── scripts/setup.sh        # one-shot setup after cloning
├── src/
│   ├── insertion/          # Challenge 1 — ROS 2 package (ament_python)
│   └── ball_balance/       # Challenge 2 — ROS 2 package (ament_python)
└── multipanda_ros2/        # driver stack, pinned as a git submodule (humble)
```
Our code lives in `src/`. The driver (`multipanda_ros2`) is a **git submodule**
pinned to a specific commit so everyone builds against the exact same version.

## Getting the repo (with the submodule)
Clone recursively so the submodule comes along:
```bash
git clone --recursive <repo-url>
cd insert-and-balance
```
Already cloned without `--recursive`? Run:
```bash
git submodule update --init --recursive
```

## One-shot setup
From the repo root, run once:
```bash
./scripts/setup.sh
```
This pulls the pinned driver and links `src/insertion` + `src/ball_balance`
into the `multipanda_ros2` colcon source tree (so `colcon build` finds them next
to `franka_bringup`). The links are marked ignored inside the submodule, so it
stays clean.

## Build & run (inside the driver's Docker container)
```bash
cd multipanda_ros2
./tools/setup_env      # builds the Docker image — first time only, slow
./run                  # opens a shell in the container, ws at ~/multipanda_ws
colcon build
source install/setup.bash
```
Extra terminal into the same container:
```bash
docker exec -it --user developer multipanda-container bash
```

### Simulation (no hardware)
```bash
ros2 launch franka_bringup franka_sim.launch.py    # MuJoCo, one Panda
```
Develop and test all logic here first — sim/real parity is the point of the stack.
Note: **Cartesian** control modes are real-robot only (not in sim); torque /
joint-position / joint-velocity work in both.

### Real arm (Black workstation, RT kernel)
First in Franka Desk: unlock joints + activate FCI. Only one client may command
the arm at a time (Desk **or** external control).
```bash
ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip>
# verify RT kernel + connection:
~/Libraries/libfranka/bin/communication_test <fci-ip>
```
After a `ControlException` (collision/reflex trip), recover with the
`~/service_server/error_recovery` service — no need to reload the controller.

## Inspect the running system
```bash
ros2 topic list
ros2 service list
ros2 action list
ros2 control list_controllers
```

## Pinned versions (must match for the driver to build)
libfranka 0.9.2 · Panda firmware 4.2.1/4.2.2 · MuJoCo 3.2.0 · **Eigen 3.3.9**
(not 3.4.0 — it breaks compilation). These are handled by the driver's Docker image.

## Updating the pinned driver
```bash
cd multipanda_ros2 && git fetch && git checkout <new-commit>
cd .. && git add multipanda_ros2 && git commit -m "Bump multipanda_ros2"
```
