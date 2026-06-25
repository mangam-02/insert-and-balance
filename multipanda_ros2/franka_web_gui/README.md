# Franka Web GUI

A modern React dashboard for controlling the Franka Panda robot over ROS2.

## Quick Start (Hackathon Demo)

### Option A — Full Docker (recommended)

```bash
cd franka_web_gui
docker compose up
```

- **Frontend**: http://localhost:5173
- **rosbridge**: ws://localhost:9090

> If rosbridge is already running on your system, comment out the `rosbridge` service.

---

### Option B — Frontend only (mock mode)

```bash
cd franka_web_gui/frontend
npm install
npm run dev
```

Open http://localhost:5173. The app runs in **mock mode** with simulated joint data — no ROS required for a demo.

---

### Option C — With existing ROS2 environment

```bash
# Terminal 1: rosbridge
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090

# Terminal 2: mock nodes (optional, for demo data)
source /opt/ros/humble/setup.bash
python3 ros2_nodes/robot_status_publisher.py

# Terminal 3: frontend
cd frontend && npm install && npm run dev
```

---

## Architecture

```
franka_web_gui/
├── frontend/
│   └── src/
│       ├── services/ros/       ← RosConnection, TopicManager, ServiceManager,
│       │                          ActionManager, CameraManager
│       ├── store/store.ts      ← Zustand global state
│       ├── hooks/
│       │   ├── useRos.ts       ← Attaches real ROS subscribers on connect
│       │   └── useMockData.ts  ← Drives simulated data when offline
│       ├── pages/              ← Dashboard, Cameras, Topics, Services,
│       │                          Actions, Skills, Logs
│       └── components/
│           ├── layout/         ← Header, Sidebar, MainLayout
│           └── dashboard/      ← RobotStatusCard, JointStateViewer, …
├── ros2_nodes/
│   ├── hello_world_publisher.py
│   ├── hello_world_subscriber.py
│   ├── robot_status_publisher.py   ← /joint_states + /robot_status at 10 Hz
│   ├── dummy_service_server.py     ← /robot/start|stop|home → always success
│   └── dummy_action_server.py      ← /franka/execute_skill → 0→100% progress
└── docker-compose.yml
```

## Topics / Services / Actions

| Channel | Type | Direction |
|---|---|---|
| `/hello_world` | `std_msgs/String` | Subscribe |
| `/hello_world_command` | `std_msgs/String` | Publish |
| `/robot_status` | `std_msgs/String` | Subscribe |
| `/joint_states` | `sensor_msgs/JointState` | Subscribe |
| `/franka_state_controller/franka_states` | `franka_msgs/FrankaState` | Subscribe |
| `/camera/color/image_compressed` | `sensor_msgs/CompressedImage` | Subscribe |
| `/camera/depth/image_compressed` | `sensor_msgs/CompressedImage` | Subscribe |
| `/robot/start` | `std_srvs/Trigger` | Service call |
| `/robot/stop` | `std_srvs/Trigger` | Service call |
| `/robot/home` | `std_srvs/Trigger` | Service call |
| `/franka/skill` | `std_msgs/String` | Publish (Skills page) |
| `/franka/execute_skill` | `FollowJointTrajectory` | Action |

## Extending for Production

1. **Real cameras**: `CameraManager.ts` already subscribes to `sensor_msgs/CompressedImage` — point your camera bridge at the configured topics.
2. **Real Franka state**: `useRos.ts` subscribes to `/franka_state_controller/franka_states` — ensure `franka_state_controller` is running.
3. **Real services**: Replace mock `DummyServiceServer` with real service nodes. The `ServiceManager` in the frontend is already wired up.
4. **Real actions**: Define custom `.action` files and swap the action type in `ActionManager.ts`.
5. **Authentication**: Add JWT or session auth to the rosbridge launch config.

## Tech Stack

- React 18 + TypeScript + Vite
- TailwindCSS (dark design system)
- Zustand (state management)
- Framer Motion (animations)
- Recharts (live joint charts)
- roslibjs / rosbridge_server (ROS2 ↔ WebSocket)
- Lucide React (icons)
