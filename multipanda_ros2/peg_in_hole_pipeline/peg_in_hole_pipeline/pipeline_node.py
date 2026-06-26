"""Peg grasp + insertion pipeline -- everything in one file, one sequential pass.

The whole pipeline is a single top-to-bottom sequence in `PipelineNode.run()`:

    HOME -> DETECT -> COMPUTE GRASP -> MOVE TO GRASP -> CLOSE GRIPPER
         -> LIFT -> COMPUTE INSERTION -> MOVE TO HOLE -> INSERT
         -> RETURN HOME -> DONE

There is no state machine and no cross-module indirection: read run() straight down
and you see exactly what happens, in order. Crucially the object poses are detected
(DETECT) BEFORE any grasp pose, marker, or motion is computed, so we never plan or
publish markers for a stale/empty pose.

Everything tunable is a ROS2 parameter (declared in `_declare_params`).

Prereqs (multipanda container, ROS_DOMAIN_ID=1):
  * MoveIt:   ros2 launch franka_moveit_config moveit.launch.py robot_ip:=... load_gripper:=true
  * Camera:   ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true ...
  * FoundationPose (other container) publishing /foundationpose/<obj>/pose
Run:
  ros2 run peg_in_hole_pipeline pipeline
  ros2 run peg_in_hole_pipeline pipeline --ros-args -p grasp_force:=30.0 -p step_through:=false
"""
import copy
import math
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped TF transform support
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

from controller_manager_msgs.srv import SwitchController
from franka_msgs.action import Grasp, Move
from franka_msgs.msg import FrankaState
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (BoundingVolume, CollisionObject, Constraints,
                             JointConstraint, OrientationConstraint,
                             PlanningScene, PositionConstraint,
                             WorkspaceParameters)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionFK


# ====================================================================================
# small math / formatting helpers
# ====================================================================================
def quat_mul(a, b):
    """Hamilton product of two quaternions.

    Input:  a, b -- quaternions as (x, y, z, w) tuples.
    Output: their product a*b as an (x, y, z, w) tuple.
    """
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_about_z(theta):
    """Quaternion for a rotation about the Z axis.

    Input:  theta -- rotation angle [rad].
    Output: (x, y, z, w) quaternion.
    """
    return (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))


def quat_about_axis(axis, theta):
    """Quaternion (x, y, z, w) for a rotation of theta [rad] about an arbitrary axis.

    Input:  axis -- length-3 vector (need not be unit); theta -- angle [rad].
    Output: (x, y, z, w) quaternion; identity if axis is ~zero length.
    """
    ax = np.asarray(axis, float)
    n = float(np.linalg.norm(ax))
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    ax = ax / n
    s = math.sin(theta / 2.0)
    return (float(ax[0] * s), float(ax[1] * s), float(ax[2] * s), math.cos(theta / 2.0))


def rotate_vec(q_xyzw, v):
    """Rotate a 3-vector by a quaternion.

    Input:  q_xyzw -- (x, y, z, w) quaternion; v -- length-3 vector.
    Output: numpy array, v rotated by q.
    """
    x, y, z, w = q_xyzw
    qv = np.array([x, y, z], float)
    v = np.asarray(v, float)
    return v + 2.0 * np.cross(qv, np.cross(qv, v) + w * v)


def fmt_pose(ps):
    """Format a PoseStamped for one-line logging.

    Input:  ps -- geometry_msgs/PoseStamped.
    Output: human-readable string with frame, position and orientation.
    """
    p, o = ps.pose.position, ps.pose.orientation
    return (f'[{ps.header.frame_id}] xyz=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) '
            f'quat=({o.x:.4f}, {o.y:.4f}, {o.z:.4f}, {o.w:.4f})')


def quat_to_mat(q_xyzw):
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix (numpy)."""
    x, y, z, w = q_xyzw
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w),     s * (x * z + y * w)],
        [s * (x * y + z * w),     1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w),     s * (y * z + x * w),     1 - s * (x * x + y * y)],
    ])


def mat_to_quat(R):
    """3x3 rotation matrix -> quaternion (x, y, z, w). Shepperd's method (robust)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] >= R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def quat_slerp(q0_xyzw, q1_xyzw, s):
    """Spherical linear interpolation between two (x, y, z, w) quaternions at fraction s."""
    a = np.asarray(q0_xyzw, float)
    b = np.asarray(q1_xyzw, float)
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    d = float(np.dot(a, b))
    if d < 0.0:                     # take the shorter arc
        b = -b
        d = -d
    if d > 0.9995:                  # nearly identical -> linear, then renormalize
        r = a + s * (b - a)
        return tuple(r / (np.linalg.norm(r) + 1e-12))
    th0 = math.acos(d)
    th = th0 * s
    s0 = math.sin(th0)
    w0 = math.sin(th0 - th) / s0
    w1 = math.sin(th) / s0
    return tuple(w0 * a + w1 * b)


def quat_angle(q0_xyzw, q1_xyzw):
    """Smallest rotation angle [rad] between two (x, y, z, w) quaternions."""
    a = np.asarray(q0_xyzw, float)
    b = np.asarray(q1_xyzw, float)
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(a, b)))))

# RViz marker colors, keyed by pose name.
MARKER_COLORS = {
    'peg':            (1.0, 0.1, 0.1, 1.0),   # red
    'hole':           (0.1, 0.4, 1.0, 1.0),   # blue
    'grasp':          (0.1, 1.0, 0.1, 1.0),   # green
    'grasp_approach': (0.1, 1.0, 1.0, 1.0),   # cyan
    'lift':           (1.0, 0.5, 0.0, 1.0),   # orange
    'hole_approach':  (1.0, 0.1, 1.0, 1.0),   # magenta
    'insertion':      (1.0, 0.9, 0.1, 1.0),   # yellow
    'tipup_approach':       (0.6, 0.9, 0.2, 1.0),   # lime
    'tipup_tilt':           (0.3, 0.9, 0.4, 1.0),   # green-cyan
    'tipup_grasp':          (0.2, 0.8, 0.2, 1.0),   # green
    'tipup_lift':           (0.9, 0.6, 0.1, 1.0),   # amber
    'tipup_clear':          (0.9, 0.8, 0.1, 1.0),   # gold
    'tipup_stand':          (0.2, 0.7, 0.9, 1.0),   # sky
    'tipup_place_approach': (0.7, 0.3, 0.9, 1.0),   # purple
    'tipup_place':          (0.9, 0.2, 0.5, 1.0),   # pink
}


class PipelineNode(Node):
    """Single ROS2 node holding every client (perception, motion, gripper, scene, markers)
    and running the peg-in-hole sequence once via run()."""

    def __init__(self):
        """Construct the node: declare parameters and create every ROS client/sub/pub.

        Input:  none.
        Output: a ready-to-run PipelineNode (call run()).
        """
        super().__init__('peg_in_hole_pipeline')
        self.p = self._declare_params()

        # --- perception (FoundationPose pose topics + TF into the base frame) ---
        self._pose_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                    history=HistoryPolicy.KEEP_LAST, depth=10)
        self._pose_subs = {}      # obj -> Subscription
        self._pose_latest = {}    # obj -> latest PoseStamped (camera frame)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # --- motion (MoveIt /move_group) ---
        self._move = ActionClient(self, MoveGroup, 'move_action')
        self._exec = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._cart = self.create_client(GetCartesianPath, 'compute_cartesian_path')

        # --- gripper (franka_gripper action servers) ---
        gns = self.p.gripper_ns.rstrip('/')
        self._grip_grasp = ActionClient(self, Grasp, f'{gns}/grasp')
        self._grip_move = ActionClient(self, Move, f'{gns}/move')
        self._grip_width = None
        self.create_subscription(JointState, f'{gns}/joint_states',
                                 self._on_gripper_js, 10)

        # --- planning scene (ground plane) ---
        self._scene = self.create_client(ApplyPlanningScene, 'apply_planning_scene')

        # --- impedance insertion (force-regulated, optional) ---
        # Switches the arm from the position controller to the Cartesian impedance controller
        # for the INSERT phase, commands an equilibrium pose, and regulates the contact force.
        self._franka_state = None     # latest franka_msgs/FrankaState (force + O_T_EE)
        self._T_tcp_ee = None         # constant tcp<-EE transform (impedance moves)
        self._home_tcp = None         # cached FK pose of home_joints
        if self.p.use_impedance_insertion or self.p.use_impedance_moves:
            self._switch_cli = self.create_client(SwitchController,
                                                  '/controller_manager/switch_controller')
            self._imp_pose_pub = self.create_publisher(
                Float64MultiArray, self.p.impedance_pose_topic, 1)
            self._imp_stiff_pub = self.create_publisher(
                Float64MultiArray, self.p.impedance_stiffness_topic, 1)
            self.create_subscription(FrankaState, self.p.franka_state_topic,
                                     self._on_franka_state, 10)
            self._fk_cli = self.create_client(GetPositionFK, 'compute_fk')
        else:
            self._switch_cli = None
            self._imp_pose_pub = None
            self._imp_stiff_pub = None
            self._fk_cli = None

        # --- execution state (latched, for the frontend) ---
        # std_msgs/String carrying the current phase name. TRANSIENT_LOCAL so a frontend
        # that subscribes mid-run immediately receives the latest state.
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.RELIABLE)
        self._state_pub = self.create_publisher(String, self.p.state_topic, state_qos)
        self._state = None
        self._set_state('IDLE')

        # --- debug markers (RViz) ---
        self._markers = {}    # name -> (PoseStamped, rgba)
        self._marker_order = []
        if self.p.debug_markers:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
            self._marker_pub = self.create_publisher(
                MarkerArray, '/peg_in_hole/debug_markers', qos)
            self.create_timer(0.5, self._publish_markers)
        else:
            self._marker_pub = None

    def _declare_params(self):
        """Declare and read every ROS2 parameter.

        Input:  none (reads the launched parameter overrides).
        Output: SimpleNamespace `p` with one attribute per parameter.
        """
        d = self.declare_parameter
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        # frames / groups / object names
        d('base_frame', 'panda_link0')
        d('planning_group', 'panda_manipulator')
        d('ee_link', 'tcp')               # 15 cm tool frame off panda_link8 (see URDF)
        d('foundationpose_ns', '/foundationpose')
        d('gripper_ns', '/panda_gripper')
        d('peg_object', 'peg')
        d('hole_object', 'insert')
        # peg orientation check (is the peg standing up or lying down?)
        d('peg_long_axis_local', [0.0, 0.0, 1.0])   # the peg's long axis in its OWN frame (FoundationPose model)
        d('peg_horizontal_threshold_deg', 45.0)     # peg long axis > this from global Z => lying down (needs tip-up)
        # tip-up regrasp: when the peg is HORIZONTAL, grasp it lying down, stand it vertical and
        # set it back down at the same spot, then re-detect and run the normal pipeline.
        # NOTE: the grasp/finger orientation conventions (tilt, finger yaw) likely need tuning on
        # the robot -- verify the poses in RViz (markers) with step_through before full speed.
        d('tipup_enabled', True)
        d('tipup_tilt_deg', 45.0)              # gripper tilt about the horizontal axis perpendicular to the peg, at grasp
        d('tipup_finger_yaw_deg', 45.0)        # extra yaw about the approach axis to align the physical fingers (mount comp)
        d('tipup_grasp_z_offset', 0.03)         # m, added to the detected peg-centre height for the lying grasp
        d('tipup_approach_height', 0.07)       # m above the grasp/place pose along base Z
        d('tipup_lift_height', 0.07)           # m to lift straight up after grasping, before standing vertical
        d('tipup_clear_z', 0.20)               # m, absolute base-Z to rise to (clear of the ground) BEFORE rotating the peg vertical
        d('tipup_place_z', 0.00)               # m, EE base-Z when setting the upright peg down (tune so the peg bottom rests on the table)
        # home + planner
        d('joint_names', ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
                          'panda_joint5', 'panda_joint6', 'panda_joint7'])
        d('home_joints', [0.561829, 0.1989799, -0.0193699, -2.865767, 0.82115, 3.720473, 0.74422])
        d('home_settle_sec', 2.0)         # dwell at HOME so the arm/camera TF settle before DETECT
        d('vel_scale', 0.4)
        d('acc_scale', 0.1)
        d('planning_time', 10.0)
        d('planning_pipeline', 'pilz')    # move_group pipeline: ompl / pilz
        d('planner_id', 'PTP')            # Pilz free-space algorithm: PTP / LIN / CIRC
        # grasp geometry (fixed top-down transform on the detected peg)
        d('grasp_orientation_xyzw', [1.0, 0.0, 0.0, 0.0])   # top-down: tool Z points down
        d('grasp_yaw_offset', 0.7853981633974483)           # +pi/4: cancels the hand's -45 deg mount
        d('grasp_offset_xyz', [0.0, 0.0, 0.0])              # m, added to peg x/y
        d('grasp_z', 0.0382)              # m, fixed grasp height in base Z (table-relative)
        d('grasp_approach_height', 0.03)  # m above the grasp, in base Z
        # gripper
        d('grasp_force', 40.0)            # N
        d('grasp_speed', 0.1)             # m/s
        d('gripper_closed_width', 0.005)  # m; final opening <= this => closed empty (no object)
        d('gripper_open_width', 0.08)
        d('open_gripper_at_end', True)
        # lift + insertion
        d('lift_height', 0.1)             # m straight up after grasp
        d('hole_approach_height', 0.13)   # m above the hole
        d('insertion_depth', 0.05)        # m straight down
        d('cartesian_step', 0.005)
        d('cartesian_min_fraction', 0.9)
        # impedance insertion (force-regulated). INSERT uses the Cartesian impedance
        # controller instead of the MoveIt Cartesian path when use_impedance_insertion=True.
        d('use_impedance_insertion', True)
        d('impedance_controller_name', 'cartesian_impedance_controller')
        d('position_controller_name', 'panda_arm_controller')
        # task-space stiffness [kx,ky,kz,krx,kry,krz] (N/m, N/m, N/m, Nm/rad...). Soft in the
        # insertion (Z) axis so contact force is gentle/controllable; damping is critical
        # (2*sqrt(k)) inside the controller -- not separately tunable.
        d('impedance_stiffness', [800.0, 800.0, 300.0, 30.0, 30.0, 30.0])
        d('franka_state_topic', '/franka_robot_state_broadcaster/robot_state')
        d('impedance_pose_topic', '/cartesian_impedance/pose_desired')
        d('impedance_stiffness_topic', '/cartesian_impedance/stiffness')
        d('insertion_target_force', 4.0)   # N, contact force the loop regulates to
        d('insertion_max_force', 20.0)      # N, |force| above this aborts (hard safety cap)
        d('insertion_force_gain', 2.0e-5)   # m/N per step, equilibrium servo gain
        d('insertion_force_sign', -1.0)     # maps o_f_ext_hat_k[2] -> +pressing force. This robot reports a downward press as NEGATIVE Fz, so -1 makes f positive while pressing (true force regulation; all force thresholds below are positive). Flip to +1 only if a press reads positive on your robot.
        d('insertion_rate_hz', 50.0)        # force-regulation loop rate
        d('insertion_timeout_sec', 20.0)    # abort if depth not reached in this time
        d('insertion_max_descent', 0.12)    # m, hard clamp on equilibrium downward travel
        # Free-space moves. With use_impedance_moves=False they run through MoveIt (planned,
        # joint/Cartesian-limit-aware, no steady-state error) -- the right tool for large
        # repositioning; only the contact INSERT uses the impedance controller. Set True to
        # route them through the impedance controller instead, but note that at the soft
        # impedance_move_stiffness below the EE settles with a large steady-state offset and
        # large reorientations can drive joints into their limits (there is NO planner).
        d('use_impedance_moves', False)
        d('impedance_home', False)          # HOME/return-home via JOINT move (MoveIt), not impedance
        d('impedance_move_stiffness', [800.0, 800.0, 300.0, 30.0, 30.0, 30.0])
        d('impedance_move_speed', 0.03)     # m/s, equilibrium ramp linear speed
        d('impedance_move_ang_speed', 0.3)  # rad/s, equilibrium ramp angular speed
        d('impedance_move_settle_pos', 0.010)  # m, converged when EE pos error below this
        d('impedance_move_settle_ang', 0.08)   # rad, converged when EE orient error below this
        d('impedance_move_settle_timeout', 7.0)  # s, wait for the EE to settle after the ramp
        # spiral search. If the peg presses on the surface instead of dropping into the hole,
        # trace an outward Archimedean spiral in x/y (under continued downward force) until the
        # peg slips in. Engages when the press force reaches the engage force (stuck on the
        # surface) and ENDS by POSITION: success once the EE has descended to within
        # spiral_search_depth_tolerance of the target insertion depth (the peg dropped into the
        # hole), otherwise it gives up after spiral_search_circles revolutions.
        d('spiral_search_enabled', True)
        d('spiral_search_circles', 20.0)          # revolutions traced from centre to max radius
        d('spiral_search_max_radius', 0.010)     # m, radius at the outermost circle
        d('spiral_search_engage_force', 3.0)     # N, press force at/above which the peg is stuck -> start spiral
        d('spiral_search_press_force', 3.0)      # N, downward press the spiral REGULATES to (force cap while searching)
        d('spiral_search_depth_tolerance', 0.005)  # m, success once descended is within this of insertion_depth (peg dropped in)
        d('spiral_search_angular_speed', 1.0)    # rad/s, how fast the spiral is traced
        # planning scene
        d('add_ground_plane', False)
        d('ground_z', -0.15)              # m, top surface of the floor in base_frame
        d('ground_size', 2.0)             # m, square extent of the floor
        # behavior
        d('state_topic', '/peg_in_hole/state')  # latched current-phase topic for the frontend
        d('debug_markers', True)          # publish colored pose arrows for RViz
        d('keep_alive', True)             # keep node + markers alive after the run
        d('step_through', False)          # if True, wait for ENTER between phases (operator stepping)
        names = ['base_frame', 'planning_group', 'ee_link', 'foundationpose_ns', 'gripper_ns',
                 'peg_object', 'hole_object', 'peg_long_axis_local', 'peg_horizontal_threshold_deg',
                 'tipup_enabled', 'tipup_tilt_deg', 'tipup_finger_yaw_deg', 'tipup_grasp_z_offset',
                 'tipup_approach_height', 'tipup_lift_height', 'tipup_clear_z', 'tipup_place_z',
                 'joint_names', 'home_joints', 'home_settle_sec',
                 'vel_scale', 'acc_scale', 'planning_time', 'planning_pipeline', 'planner_id',
                 'grasp_orientation_xyzw', 'grasp_yaw_offset', 'grasp_offset_xyz', 'grasp_z',
                 'grasp_approach_height', 'grasp_force', 'grasp_speed', 'gripper_closed_width',
                 'gripper_open_width', 'open_gripper_at_end', 'lift_height',
                 'hole_approach_height', 'insertion_depth', 'cartesian_step',
                 'cartesian_min_fraction', 'use_impedance_insertion',
                 'impedance_controller_name', 'position_controller_name', 'impedance_stiffness',
                 'franka_state_topic', 'impedance_pose_topic', 'impedance_stiffness_topic',
                 'insertion_target_force', 'insertion_max_force', 'insertion_force_gain',
                 'insertion_force_sign', 'insertion_rate_hz', 'insertion_timeout_sec',
                 'insertion_max_descent',
                 'use_impedance_moves', 'impedance_home', 'impedance_move_stiffness',
                 'impedance_move_speed', 'impedance_move_ang_speed',
                 'impedance_move_settle_pos', 'impedance_move_settle_ang',
                 'impedance_move_settle_timeout',
                 'spiral_search_enabled', 'spiral_search_circles', 'spiral_search_max_radius',
                 'spiral_search_engage_force', 'spiral_search_press_force',
                 'spiral_search_depth_tolerance', 'spiral_search_angular_speed',
                 'add_ground_plane', 'ground_z', 'ground_size',
                 'state_topic', 'debug_markers', 'keep_alive', 'step_through']
        return SimpleNamespace(**{n: g(n) for n in names})

    # ================================================================================
    # infra waits + small utilities
    # ================================================================================
    def _wait_for_infra(self, timeout_sec=20.0):
        """Block until MoveIt and gripper action servers are available.

        Input:  timeout_sec -- per-server wait budget [s].
        Output: True if all servers came up, else False (with an error logged).
        """
        log = self.get_logger()
        log.info('Waiting for MoveIt + gripper action servers...')
        ok = self._move.wait_for_server(timeout_sec=timeout_sec)
        ok &= self._exec.wait_for_server(timeout_sec=timeout_sec)
        ok &= self._cart.wait_for_service(timeout_sec=timeout_sec)
        if not ok:
            log.error('MoveIt servers unavailable — is moveit.launch.py running?')
            return False
        if not (self._grip_grasp.wait_for_server(timeout_sec=10.0)
                and self._grip_move.wait_for_server(timeout_sec=10.0)):
            log.error('gripper servers unavailable — is franka_gripper running (load_gripper:=true)?')
            return False
        return True

    def _dwell(self, seconds):
        """Sleep while still spinning the node, so TF/pose subscriptions keep updating.

        Input:  seconds -- dwell time [s]; <=0 returns immediately.
        Output: none.
        """
        if seconds <= 0.0:
            return
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _set_state(self, state):
        """Publish the current execution phase on the latched state topic.

        Input:  state -- short phase name (e.g. 'DETECT', 'INSERT', 'DONE', 'ABORTED').
        Output: none. Stores and publishes the state so the frontend stays in sync.
        """
        self._state = state
        self._state_pub.publish(String(data=state))

    def _pause(self, done, nxt):
        """Wait for ENTER between phases when step_through is on (operator inspection).

        Input:  done -- name of the finished phase; nxt -- name of the next phase.
        Output: none. Blocks on stdin; no stdin (EOF) just continues.
        """
        if not self.p.step_through:
            return
        try:
            input(f'\n[step] finished {done}. ENTER to run {nxt} (Ctrl-C aborts) ... ')
        except EOFError:
            self.get_logger().warn('step-through: no stdin — continuing without pausing')

    # ================================================================================
    # perception
    # ================================================================================
    def get_object_pose(self, obj, timeout_sec=5.0):
        """Detect one object via FoundationPose and return its pose in the base frame.

        Subscribes to <foundationpose_ns>/<obj>/pose (camera frame), waits for a FRESH
        message, then transforms it into base_frame via TF (the hand-eye calibration).

        Input:  obj -- object name (e.g. 'peg'); timeout_sec -- wait budget [s].
        Output: geometry_msgs/PoseStamped in base_frame, or None on timeout/TF failure.
        """
        log = self.get_logger()
        if obj not in self._pose_subs:
            topic = f"{self.p.foundationpose_ns.rstrip('/')}/{obj}/pose"
            self._pose_subs[obj] = self.create_subscription(
                PoseStamped, topic,
                lambda msg, o=obj: self._pose_latest.__setitem__(o, msg), self._pose_qos)
        # Drop any stale pose so we block until a genuinely fresh frame arrives.
        self._pose_latest.pop(obj, None)
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and obj not in self._pose_latest:
            if self.get_clock().now().nanoseconds > deadline:
                log.error(f'no pose on {self.p.foundationpose_ns}/{obj}/pose within '
                          f'{timeout_sec:.1f}s (is FoundationPose publishing for "{obj}"?)')
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        ps_cam = self._pose_latest[obj]
        # Transform camera-frame pose into the base frame.
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok():
            try:
                return self._tf_buffer.transform(
                    ps_cam, self.p.base_frame, timeout=rclpy.duration.Duration(seconds=0.2))
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException) as exc:
                if self.get_clock().now().nanoseconds > deadline:
                    log.error(f'{obj}: TF {ps_cam.header.frame_id} -> {self.p.base_frame} '
                              f'failed: {exc}')
                    return None
                rclpy.spin_once(self, timeout_sec=0.1)

    # ================================================================================
    # grasp / insertion geometry  (pure functions of a detected object pose)
    # ================================================================================
    def _grasp_quat(self):
        """Commanded tool orientation for grasp + insertion.

        Input:  none (uses grasp_orientation_xyzw + grasp_yaw_offset params).
        Output: (x, y, z, w) quaternion: top-down, yaw-offset to align fingers to link8.
        """
        return quat_mul(tuple(self.p.grasp_orientation_xyzw),
                        quat_about_z(self.p.grasp_yaw_offset))

    def classify_peg_orientation(self, peg_ps):
        """Decide whether the detected peg is standing up (vertical) or lying down (horizontal).

        Rotates the peg's own long axis (peg_long_axis_local) by the detected orientation into
        base_frame and measures its angle to the global Z axis.

        Input:  peg_ps -- peg PoseStamped in base_frame.
        Output: (is_horizontal, tilt_deg, axis_base):
                  is_horizontal -- True if that angle exceeds peg_horizontal_threshold_deg;
                  tilt_deg      -- angle [deg] between the peg long axis and global Z (0 = upright);
                  axis_base     -- unit peg long axis in base_frame (numpy length-3).
        """
        o = peg_ps.pose.orientation
        a = rotate_vec((o.x, o.y, o.z, o.w), self.p.peg_long_axis_local)
        a = a / max(1e-9, float(np.linalg.norm(a)))
        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(a[2]))))))
        is_horizontal = tilt_deg > float(self.p.peg_horizontal_threshold_deg)
        self.get_logger().info(
            f'PEG ORIENTATION: long axis = ({a[0]:.3f}, {a[1]:.3f}, {a[2]:.3f}) in base | '
            f'tilt from vertical = {tilt_deg:.1f} deg -> '
            f'{"HORIZONTAL (needs tip-up)" if is_horizontal else "VERTICAL (ok)"}')
        return is_horizontal, tilt_deg, a

    @staticmethod
    def _quat_tool_from_axes(z_axis, x_hint):
        """Tool quaternion (x,y,z,w) whose +Z is z_axis and +X is x_hint projected perpendicular
        to Z (right-handed: Y = Z x X). Used to aim the approach (Z) and align a lateral axis."""
        z = np.asarray(z_axis, float)
        z = z / (np.linalg.norm(z) + 1e-12)
        x = np.asarray(x_hint, float)
        x = x - z * float(np.dot(x, z))
        if np.linalg.norm(x) < 1e-9:                       # x_hint parallel to z -> pick any perp
            x = np.array([1.0, 0.0, 0.0]) - z * float(z[0])
        x = x / (np.linalg.norm(x) + 1e-12)
        y = np.cross(z, x)
        return mat_to_quat(np.column_stack((x, y, z)))

    def compute_tipup(self, peg_ps, axis_base):
        """Build the tip-up pose sequence for a lying peg (all PoseStamped in base_frame).

        Grasp: tilted top-down over the peg, fingers across the cylinder (perpendicular to its
        horizontal axis). Stand: the whole grasped peg is rotated so its long axis points straight
        DOWN (vertical), by pre-multiplying a base-frame rotation that maps the peg axis to -Z onto
        the grasp orientation -- so the peg ends vertical regardless of the grasp tilt. Place: set
        it back down at the same x/y, upright.

        Input:  peg_ps -- detected (lying) peg PoseStamped; axis_base -- unit peg long axis in base.
        Output: dict name -> PoseStamped for each waypoint.
        """
        p = self.p
        px, py, pz = peg_ps.pose.position.x, peg_ps.pose.position.y, peg_ps.pose.position.z
        a = np.asarray(axis_base, float)
        a = a / (np.linalg.norm(a) + 1e-12)
        h = np.array([a[0], a[1], 0.0])                    # cylinder direction in the table plane
        if np.linalg.norm(h) < 1e-6:
            h = np.array([1.0, 0.0, 0.0])
        h = h / np.linalg.norm(h)
        # Straight top-down approach orientation (fingers across the cylinder). The robot moves
        # horizontally to this pose FIRST, then tilts in place before descending.
        q_top = quat_mul(self._quat_tool_from_axes([0.0, 0.0, -1.0], h),
                         quat_about_z(math.radians(p.tipup_finger_yaw_deg)))
        # Tilted approach: lean the straight-down approach by tipup_tilt_deg along the peg axis,
        # AWAY from the base, so it points down-and-out toward -Z (negative tilt leans the other
        # way). Horizontal lean direction = the peg axis pointing away from the base origin.
        r = np.array([px, py, 0.0])
        d_lean = h if float(np.dot(h, r)) >= 0.0 else -h
        th = math.radians(p.tipup_tilt_deg)
        z_app = -np.array([0.0, 0.0, 1.0]) * math.cos(th) + d_lean * math.sin(th)
        q_grasp = quat_mul(self._quat_tool_from_axes(z_app, h),
                           quat_about_z(math.radians(p.tipup_finger_yaw_deg)))
        # stand upright: base-frame rotation taking the peg axis a -> straight down, applied to q_grasp
        angle = math.acos(max(-1.0, min(1.0, float(np.dot(a, [0.0, 0.0, -1.0])))))
        q_align = quat_about_axis(np.cross(a, [0.0, 0.0, -1.0]), angle)
        q_stand = quat_mul(q_align, q_grasp)
        gz = pz + float(p.tipup_grasp_z_offset)

        def mk(x, y, z, q):
            ps = PoseStamped()
            ps.header.frame_id = peg_ps.header.frame_id
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), float(z)
            (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w) = (float(q[0]), float(q[1]),
                                                              float(q[2]), float(q[3]))
            return ps

        return {
            'tipup_approach':       mk(px, py, gz + p.tipup_approach_height, q_top),    # top-down, untilted
            'tipup_tilt':           mk(px, py, gz + p.tipup_approach_height, q_grasp),  # tilt in place
            'tipup_grasp':          mk(px, py, gz, q_grasp),
            'tipup_lift':           mk(px, py, gz + p.tipup_lift_height, q_grasp),
            'tipup_clear':          mk(px, py, p.tipup_clear_z, q_grasp),   # rise clear of the ground (still horizontal)
            'tipup_stand':          mk(px, py, p.tipup_clear_z, q_stand),   # align vertical up high
            'tipup_place_approach': mk(px, py, p.tipup_place_z + p.tipup_approach_height, q_stand),
            'tipup_place':          mk(px, py, p.tipup_place_z, q_stand),
        }

    def tip_up_peg(self, peg_ps, axis_base):
        """Stand a lying peg upright: grasp it tilted, lift, rotate to vertical, set it down at the
        same x/y, release and retract. Leaves the peg standing for a fresh DETECT.

        Input:  peg_ps -- detected lying peg; axis_base -- unit peg long axis in base.
        Output: True on success; False (via _abort) on any motion/grasp failure.
        """
        log = self.get_logger()
        log.info('==> TIP-UP: peg is lying down; standing it upright before grasp.')
        poses = self.compute_tipup(peg_ps, axis_base)
        for name, ps in poses.items():
            self.show_marker(name, ps)
            log.info(f'  {name:22s}: {fmt_pose(ps)}')
        self._pause('TIP-UP COMPUTE', 'TIP-UP EXECUTE')   # inspect the poses in RViz first
        # The tip-up grasps and places the peg right at the table, so the ground_plane collision
        # object makes every plan invalid. Drop it for the maneuver and ALWAYS restore it after
        # (the normal grasp/insert that follows still wants the floor).
        if self.p.add_ground_plane:
            self.remove_ground_plane()
        try:
            seq = [('tipup_approach', 'tip-up approach (top-down)'),
                   ('tipup_tilt', 'tip-up tilt in place'),
                   ('tipup_grasp', 'tip-up grasp')]
            for key, label in seq:
                ok, code = self.goto_pose(poses[key], label)
                if not ok:
                    return self._abort('TIP-UP', f'{label} failed ({code})')
            held, width, msg = self.gripper_force_grasp()
            wtxt = f'{width:.4f} m' if width is not None else 'unknown'
            log.info(f'tip-up grasp: held={held}, width={wtxt}, msg="{msg}"')
            if not held:
                return self._abort('TIP-UP', f'no peg grasped (width={wtxt})')
            for key, label in [('tipup_lift', 'tip-up lift'),
                               ('tipup_clear', 'tip-up clear ground'),
                               ('tipup_stand', 'tip-up stand vertical'),
                               ('tipup_place_approach', 'tip-up place approach'),
                               ('tipup_place', 'tip-up place')]:
                ok, code = self.goto_pose(poses[key], label)
                if not ok:
                    return self._abort('TIP-UP', f'{label} failed ({code})')
            self.gripper_open()                           # release the now-upright peg
            ok, code = self.goto_pose(poses['tipup_place_approach'], 'tip-up retract')
            if not ok:
                log.warn(f'tip-up retract failed ({code}); continuing')
            return True
        finally:
            if self.p.add_ground_plane:
                self.add_ground_plane()                   # restore the floor for the rest of the run

    def compute_grasp(self, peg_ps):
        """Build the grasp and approach poses from the detected peg pose.

        Input:  peg_ps -- peg PoseStamped in base_frame.
        Output: (approach_ps, grasp_ps) PoseStamped in base_frame. grasp_ps is a fixed
                top-down pose at the peg x/y (+offset) and a fixed table-relative height;
                approach_ps is grasp_ps raised by grasp_approach_height in base Z.
        """
        log = self.get_logger()
        ox, oy, _ = self.p.grasp_offset_xyz
        qx, qy, qz, qw = self._grasp_quat()
        grasp = PoseStamped()
        grasp.header.frame_id = peg_ps.header.frame_id
        grasp.header.stamp = self.get_clock().now().to_msg()
        grasp.pose.position.x = peg_ps.pose.position.x + ox
        grasp.pose.position.y = peg_ps.pose.position.y + oy
        grasp.pose.position.z = self.p.grasp_z
        grasp.pose.orientation.x, grasp.pose.orientation.y = qx, qy
        grasp.pose.orientation.z, grasp.pose.orientation.w = qz, qw
        approach = copy.deepcopy(grasp)
        approach.pose.position.z += self.p.grasp_approach_height
        log.info('--- GRASP GENERATION ---')
        log.info(f'peg pose      : {fmt_pose(peg_ps)}')
        log.info(f'grasp pose    : {fmt_pose(grasp)}')
        log.info(f'approach pose : {fmt_pose(approach)}')
        return approach, grasp

    def compute_insertion(self, hole_ps):
        """Build the hole-approach and insertion poses from the detected hole pose.

        Input:  hole_ps -- hole PoseStamped in base_frame.
        Output: (approach_ps, insertion_ps) PoseStamped in base_frame. approach_ps sits
                hole_approach_height above the hole; insertion_ps is straight down from it
                by insertion_depth. Both share the grasp tool orientation.
        """
        log = self.get_logger()
        qx, qy, qz, qw = self._grasp_quat()
        approach = PoseStamped()
        approach.header.frame_id = hole_ps.header.frame_id
        approach.header.stamp = self.get_clock().now().to_msg()
        approach.pose.position.x = hole_ps.pose.position.x
        approach.pose.position.y = hole_ps.pose.position.y
        approach.pose.position.z = hole_ps.pose.position.z + self.p.hole_approach_height
        approach.pose.orientation.x, approach.pose.orientation.y = qx, qy
        approach.pose.orientation.z, approach.pose.orientation.w = qz, qw
        insertion = copy.deepcopy(approach)
        insertion.pose.position.z -= self.p.insertion_depth
        log.info('--- INSERTION GENERATION ---')
        log.info(f'hole pose      : {fmt_pose(hole_ps)}')
        log.info(f'approach pose  : {fmt_pose(approach)}')
        log.info(f'insertion pose : {fmt_pose(insertion)}')
        return approach, insertion

    # ================================================================================
    # motion  (MoveIt /move_group, no moveit_py dependency)
    # ================================================================================
    def _base_request(self, req):
        """Fill the shared MoveGroup planning fields (group, planner, scaling, workspace).

        Input:  req -- a moveit_msgs MotionPlanRequest to populate in place.
        Output: none (mutates req).
        """
        req.group_name = self.p.planning_group
        req.pipeline_id = self.p.planning_pipeline
        req.planner_id = self.p.planner_id
        req.num_planning_attempts = 10
        req.allowed_planning_time = self.p.planning_time
        req.max_velocity_scaling_factor = self.p.vel_scale
        req.max_acceleration_scaling_factor = self.p.acc_scale
        ws = WorkspaceParameters()
        ws.header.frame_id = self.p.base_frame
        ws.min_corner = Vector3(x=-1.5, y=-1.5, z=-1.5)
        ws.max_corner = Vector3(x=1.5, y=1.5, z=1.5)
        req.workspace_parameters = ws

    def _send_move_goal(self, goal, timeout_sec=60.0):
        """Send a MoveGroup goal and block until it finishes (plan + execute).

        Input:  goal -- MoveGroup.Goal; timeout_sec -- per-phase wait budget [s].
        Output: (ok, code): ok True iff MoveItErrorCodes.SUCCESS; code is the raw int.
        """
        send = self._move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=timeout_sec)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        wrapper = result_future.result()
        if wrapper is None:
            return False, None
        code = wrapper.result.error_code.val      # SUCCESS == 1
        return code == 1, code

    def move_to_pose(self, pose, timeout_sec=60.0):
        """Free-space plan + execute to a Cartesian goal pose.

        Input:  pose -- PoseStamped goal in base_frame; timeout_sec -- wait budget [s].
        Output: (ok, code) as in _send_move_goal.
        """
        goal = MoveGroup.Goal()
        self._base_request(goal.request)
        pc = PositionConstraint()
        pc.header.frame_id = self.p.base_frame
        pc.link_name = self.p.ee_link
        pc.weight = 1.0
        region = BoundingVolume()
        region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005]))
        region.primitive_poses.append(pose.pose)
        pc.constraint_region = region
        oc = OrientationConstraint()
        oc.header.frame_id = self.p.base_frame
        oc.link_name = self.p.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.005
        oc.absolute_y_axis_tolerance = 0.005
        oc.absolute_z_axis_tolerance = 0.005
        oc.weight = 1.0
        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        goal.request.goal_constraints.append(c)
        goal.planning_options.plan_only = False
        return self._send_move_goal(goal, timeout_sec)

    def move_to_joints(self, joint_names, positions, tolerance=0.005, timeout_sec=60.0):
        """Free-space plan + execute to a joint configuration (used for HOME).

        Input:  joint_names + positions -- target joints; tolerance -- per-joint [rad].
        Output: (ok, code) as in _send_move_goal.
        """
        goal = MoveGroup.Goal()
        self._base_request(goal.request)
        c = Constraints()
        for name, pos in zip(joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)
        goal.planning_options.plan_only = False
        return self._send_move_goal(goal, timeout_sec)

    def move_cartesian(self, waypoints, timeout_sec=60.0):
        """Plan a straight-line path through waypoints and execute it (lift / insertion).

        Input:  waypoints -- list of geometry_msgs/Pose in base_frame.
        Output: (ok, fraction): ok True iff executed and >= cartesian_min_fraction of the
                path was planned; fraction is the planned fraction in [0, 1].
        """
        req = GetCartesianPath.Request()
        req.header.frame_id = self.p.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.p.planning_group
        req.link_name = self.p.ee_link
        req.waypoints = list(waypoints)
        req.max_step = float(self.p.cartesian_step)
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        # Newer moveit_msgs carry velocity scaling on the request; the multipanda fork does
        # not, so when absent we re-time the returned trajectory below instead.
        scale_on_request = hasattr(req, 'max_velocity_scaling_factor')
        if scale_on_request:
            req.max_velocity_scaling_factor = float(self.p.vel_scale)
            req.max_acceleration_scaling_factor = float(self.p.acc_scale)
        future = self._cart.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        resp = future.result()
        if resp is None:
            return False, 0.0
        fraction = resp.fraction
        if resp.error_code.val != 1 or fraction < self.p.cartesian_min_fraction:
            return False, fraction
        solution = resp.solution
        if not scale_on_request:
            self._scale_trajectory_time(solution, self.p.vel_scale)
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = solution
        send = self._exec.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=timeout_sec)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, fraction
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        wrapper = result_future.result()
        ok = wrapper is not None and wrapper.result.error_code.val == 1
        return ok, fraction

    @staticmethod
    def _scale_trajectory_time(robot_traj, vel_scale):
        """Slow a trajectory by stretching timestamps by 1/vel_scale (older moveit_msgs).

        Input:  robot_traj -- moveit_msgs/RobotTrajectory; vel_scale in (0, 1].
        Output: the same robot_traj, re-timed in place (>=1 is a no-op).
        """
        v = float(vel_scale)
        if v <= 0.0 or v >= 1.0:
            return robot_traj
        s = 1.0 / v
        for pt in robot_traj.joint_trajectory.points:
            total_ns = (pt.time_from_start.sec * 1_000_000_000 + pt.time_from_start.nanosec) * s
            pt.time_from_start.sec = int(total_ns // 1_000_000_000)
            pt.time_from_start.nanosec = int(total_ns % 1_000_000_000)
            pt.velocities = [val * v for val in pt.velocities]
            pt.accelerations = [val * v * v for val in pt.accelerations]
        return robot_traj

    # ================================================================================
    # gripper  (franka_gripper action servers)
    # ================================================================================
    def _on_gripper_js(self, msg):
        """Track the live finger opening from the gripper joint_states.

        Input:  msg -- sensor_msgs/JointState from the gripper.
        Output: none (updates self._grip_width with the summed finger opening [m]).
        """
        w, found = 0.0, False
        for name, pos in zip(msg.name, msg.position):
            if 'finger_joint' in name:
                w += pos
                found = True
        if found:
            self._grip_width = w

    def _run_gripper(self, client, goal, timeout_sec):
        """Send a gripper action goal and block for its result.

        Input:  client -- ActionClient; goal -- its goal msg; timeout_sec [s].
        Output: (success, msg): success per the action result; msg is the error string.
        """
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=timeout_sec)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, 'goal rejected'
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        wrapper = result_future.result()
        if wrapper is None:
            return False, 'no result (timeout)'
        res = wrapper.result
        return bool(getattr(res, 'success', False)), getattr(res, 'error', '')

    def gripper_open(self, width=None, speed=0.1, timeout_sec=15.0):
        """Open the gripper to a target width.

        Input:  width -- opening [m] (default gripper_open_width); speed [m/s].
        Output: (success, msg).
        """
        width = self.p.gripper_open_width if width is None else width
        goal = Move.Goal()
        goal.width = float(width)
        goal.speed = float(speed)
        self.get_logger().info(f'GRIPPER open: width={width:.4f} m')
        return self._run_gripper(self._grip_move, goal, timeout_sec)

    def gripper_force_grasp(self, timeout_sec=15.0):
        """Close fully with grasp_force, then check whether an object is held.

        Commands a full close (width 0) at grasp_force; if the fingers end up closed
        completely (final width <= gripper_closed_width) nothing was grasped.

        Input:  none (uses grasp_force / grasp_speed / gripper_closed_width params).
        Output: (held, width, msg): held True only if an object stopped the fingers;
                width is the measured final opening [m] (or None if unreadable).
        """
        goal = Grasp.Goal()
        goal.width = 0.0
        goal.speed = float(self.p.grasp_speed)
        goal.force = float(self.p.grasp_force)
        goal.epsilon.inner = 0.0
        goal.epsilon.outer = 0.1   # wide: action reports success at whatever width it stops
        self.get_logger().info(
            f'GRIPPER force-grasp: full close, force={self.p.grasp_force:.1f} N')
        ok, msg = self._run_gripper(self._grip_grasp, goal, timeout_sec)
        # Refresh the measured width.
        self._grip_width = None
        deadline = self.get_clock().now().nanoseconds + int(1.0 * 1e9)
        while rclpy.ok() and self._grip_width is None and \
                self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        width = self._grip_width
        if width is None:
            self.get_logger().warn('GRIPPER force-grasp: no finger joint_states — cannot verify')
            return ok, None, msg or 'no joint_states'
        held = ok and width > float(self.p.gripper_closed_width)
        return held, width, msg

    # ================================================================================
    # impedance insertion (force-regulated, via the Cartesian impedance controller)
    # ================================================================================
    def _on_franka_state(self, msg):
        """Cache the latest franka_msgs/FrankaState (carries O_T_EE and o_f_ext_hat_k)."""
        self._franka_state = msg

    def _wait_franka_state(self, timeout_sec=3.0):
        """Block (while spinning) for a fresh FrankaState, or None on timeout."""
        self._franka_state = None
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and self._franka_state is None:
            if self.get_clock().now().nanoseconds > deadline:
                return None
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._franka_state

    @staticmethod
    def _otee_to_pose(t):
        """Decompose a column-major 4x4 O_T_EE (length-16) into (x, y, z, R_rowmajor[9])."""
        x, y, z = t[12], t[13], t[14]              # translation = column 3
        r = [t[0], t[4], t[8],                     # R(i,j) = t[4*j + i] -> flatten row-major
             t[1], t[5], t[9],
             t[2], t[6], t[10]]
        return float(x), float(y), float(z), [float(v) for v in r]

    def _pub_impedance_pose(self, x, y, z, r9):
        """Publish an equilibrium pose to the impedance controller.

        Layout expected by CartesianImpedanceController: [x, y, z, R00..R22] (row-major 3x3).
        """
        msg = Float64MultiArray()
        msg.data = [float(x), float(y), float(z)] + [float(v) for v in r9]
        self._imp_pose_pub.publish(msg)

    def _pub_impedance_stiffness(self, k6):
        """Publish task-space stiffness [kx,ky,kz,krx,kry,krz]; controller critically damps it."""
        msg = Float64MultiArray()
        msg.data = [float(v) for v in k6]
        self._imp_stiff_pub.publish(msg)

    def _switch_controllers(self, activate, deactivate, strictness=None, timeout_sec=5.0):
        """Activate/deactivate controllers via /controller_manager/switch_controller.

        Input:  activate/deactivate -- lists of controller names; strictness -- STRICT by
                default (BEST_EFFORT for cleanup so an already-stopped controller is fine).
        Output: True iff the service reported success.
        """
        if self._switch_cli is None:
            return False
        if strictness is None:
            strictness = SwitchController.Request.STRICT
        if not self._switch_cli.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('switch_controller service unavailable '
                                    '(is controller_manager running?)')
            return False
        req = SwitchController.Request()
        # Humble uses activate_/deactivate_; older distros use start_/stop_.
        if hasattr(req, 'activate_controllers'):
            req.activate_controllers = list(activate)
            req.deactivate_controllers = list(deactivate)
        else:                                          # pragma: no cover - distro fallback
            req.start_controllers = list(activate)
            req.stop_controllers = list(deactivate)
        req.strictness = strictness
        if hasattr(req, 'activate_asap'):
            req.activate_asap = True
        future = self._switch_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        resp = future.result()
        ok = bool(resp and resp.ok)
        self.get_logger().info(f'switch_controller +{activate} -{deactivate}: '
                               f'{"OK" if ok else "FAILED"}')
        return ok

    def impedance_insert(self):
        """Force-regulated insertion with the Cartesian impedance controller.

        Hands the arm from the position controller to the impedance controller, then servos
        the equilibrium pose straight down in base Z from the current end-effector pose to
        regulate the contact force (o_f_ext_hat_k Z) toward insertion_target_force, advancing
        until the EE has descended insertion_depth. The equilibrium x/y/orientation are held
        at the captured start pose (frame-agnostic: uses the controller's own O_T_EE, not the
        MoveIt tcp frame). Aborts if |force| exceeds insertion_max_force, and ALWAYS restores
        the position controller before returning.

        Spiral search (spiral_search_enabled): if the peg presses on the surface without
        descending (press force at/above spiral_search_engage_force), an outward Archimedean
        spiral in x/y is added to the equilibrium pose -- spiral_search_circles revolutions out
        to spiral_search_max_radius -- while a gentle press (spiral_search_press_force) is held.
        The search ends by POSITION: once the peg drops into the hole and the EE descends to
        within spiral_search_depth_tolerance of the target insertion depth, INSERT succeeds (the
        caller then opens the gripper). Tracing the whole spiral without reaching depth aborts.

        Input:  none (uses the impedance/insertion_* params).
        Output: (ok, reason).
        """
        log = self.get_logger()
        p = self.p
        if self._switch_cli is None:
            return False, 'impedance insertion disabled (use_impedance_insertion=False)'
        if self._wait_franka_state(timeout_sec=3.0) is None:
            return False, (f'no FrankaState on {p.franka_state_topic} -- is '
                           f'franka_robot_state_broadcaster running?')
        if not self._switch_controllers(activate=[p.impedance_controller_name],
                                        deactivate=[p.position_controller_name]):
            return False, 'could not activate the impedance controller'
        try:
            st = self._wait_franka_state(timeout_sec=2.0)
            if st is None:
                return False, 'lost FrankaState after switching controllers'
            x0, y0, z0, r9 = self._otee_to_pose(st.o_t_ee)
            desired_z = z0
            self._pub_impedance_stiffness(p.impedance_stiffness)
            self._pub_impedance_pose(x0, y0, desired_z, r9)     # hold the captured pose first
            log.info(f'IMPEDANCE INSERT: start z={z0:.4f} m | target F={p.insertion_target_force:.1f} N '
                     f'| max F={p.insertion_max_force:.1f} N | depth {p.insertion_depth*1000:.0f} mm '
                     f'| K={list(p.impedance_stiffness)}')
            period = 1.0 / max(1.0, float(p.insertion_rate_hz))
            deadline = self.get_clock().now().nanoseconds + int(p.insertion_timeout_sec * 1e9)
            log_every = max(1, int(p.insertion_rate_hz / 5.0))   # ~5 Hz console logging
            # spiral search state: once the peg is pressing on the surface (stuck), `searching`
            # turns on and (sx, sy) trace an outward Archimedean spiral added to the equilibrium
            # x/y. theta runs 0 -> theta_max; radius grows linearly with theta to max_radius.
            searching = False
            theta = 0.0
            theta_max = 2.0 * math.pi * float(p.spiral_search_circles)
            dtheta = float(p.spiral_search_angular_speed) * period
            sx = sy = 0.0
            i = 0
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=period)
                st = self._franka_state
                if st is None:
                    continue
                f_raw = float(st.o_f_ext_hat_k[2])               # base-frame Fz
                f = float(p.insertion_force_sign) * f_raw        # +ve while pressing into hole
                cur_z = float(st.o_t_ee[14])
                descended = z0 - cur_z
                i += 1
                if i % log_every == 0:
                    extra = f'  spiral r={math.hypot(sx, sy)*1000:.1f} mm' if searching else ''
                    log.info(f'  insert: depth={descended*1000:6.1f} mm  F={f:6.1f} N  '
                             f'z_eq={desired_z:.4f}{extra}')
                if abs(f_raw) > p.insertion_max_force:            # hard safety cap
                    return False, (f'ABORT: |F|={abs(f_raw):.1f} N > max '
                                   f'{p.insertion_max_force:.1f} N at depth {descended*1000:.1f} mm')
                if descended >= p.insertion_depth:               # success: reached full depth
                    return True, f'inserted {descended*1000:.1f} mm at F={f:.1f} N'
                # spiral search: engage when the peg presses hard on the surface without
                # descending. f is +ve while pressing (insertion_force_sign), so the engage
                # threshold is +ve N.
                if p.spiral_search_enabled and not searching and f >= p.spiral_search_engage_force:
                    searching = True
                    theta = 0.0
                    log.info(f'  spiral search: engaged at F={f:.1f} N '
                             f'(>= {p.spiral_search_engage_force:.1f} N) — tracing '
                             f'{p.spiral_search_circles:.0f} circles out to '
                             f'r={p.spiral_search_max_radius*1000:.1f} mm')
                if searching:
                    # End by POSITION, not force: success once the peg has dropped to within
                    # spiral_search_depth_tolerance of the target insertion depth; otherwise give
                    # up after the whole spiral (circle limit) is traced.
                    if p.insertion_depth - descended <= p.spiral_search_depth_tolerance:
                        return True, (f'hole found via spiral search: descended '
                                      f'{descended*1000:.1f} mm (within '
                                      f'{p.spiral_search_depth_tolerance*1000:.1f} mm of '
                                      f'{p.insertion_depth*1000:.0f} mm), '
                                      f'r={math.hypot(sx, sy)*1000:.1f} mm')
                    if theta >= theta_max:                        # spiral exhausted, give up
                        return False, (f'spiral search exhausted '
                                       f'({p.spiral_search_circles:.0f} circles, '
                                       f'r={p.spiral_search_max_radius*1000:.1f} mm) '
                                       f'without finding the hole')
                    theta += dtheta
                    radius = p.spiral_search_max_radius * (theta / theta_max)
                    sx = radius * math.cos(theta)
                    sy = radius * math.sin(theta)
                elif self.get_clock().now().nanoseconds > deadline:
                    return False, (f'timeout: depth {descended*1000:.1f} mm < '
                                   f'{p.insertion_depth*1000:.0f} mm (F={f:.1f} N)')
                # force regulation: servo the equilibrium so the press force -> target. While
                # searching, regulate to the (gentler) spiral press force so the search keeps a
                # limited, capped contact instead of grinding the peg into the surface.
                target_f = p.spiral_search_press_force if searching else p.insertion_target_force
                err = float(target_f) - f
                desired_z -= float(p.insertion_force_gain) * err
                desired_z = max(desired_z, z0 - p.insertion_max_descent)   # clamp travel
                self._pub_impedance_pose(x0 + sx, y0 + sy, desired_z, r9)
            return False, 'interrupted'
        finally:
            # Always hand control back to the position controller (BEST_EFFORT so an already
            # inactive impedance controller doesn't make this error).
            self._switch_controllers(activate=[p.position_controller_name],
                                     deactivate=[p.impedance_controller_name],
                                     strictness=SwitchController.Request.BEST_EFFORT)

    # ================================================================================
    # impedance free-space moves (high stiffness, speed-limited equilibrium ramp)
    # ================================================================================
    def _pose_to_T(self, pose):
        """geometry_msgs/Pose -> 4x4 homogeneous transform (numpy)."""
        T = np.eye(4)
        o = pose.orientation
        T[:3, :3] = quat_to_mat((o.x, o.y, o.z, o.w))
        T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        return T

    def _read_otee_T(self):
        """Current O_T_EE as a 4x4 (from the cached FrankaState), or None."""
        st = self._franka_state
        if st is None:
            return None
        return np.array(st.o_t_ee, float).reshape((4, 4), order='F')   # o_t_ee is column-major

    def _T_to_cmd(self, T):
        """4x4 EE transform -> impedance pose message [x, y, z, R00..R22] (row-major 3x3)."""
        t = [float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]
        return t + [float(v) for v in T[:3, :3].reshape(-1)]

    def _ensure_tcp_ee_calib(self):
        """Compute (once) the constant tcp<-EE transform so tcp-frame targets can be turned
        into O_T_EE equilibrium poses for the impedance controller. Uses the current O_T_EE
        (FrankaState) and TF base->tcp; both are rigid to the wrist so the result is constant.
        """
        if self._T_tcp_ee is not None:
            return True
        if self._wait_franka_state(timeout_sec=3.0) is None:
            self.get_logger().error('tcp<-EE calib: no FrankaState')
            return False
        T_base_ee = self._read_otee_T()
        try:
            tf = self._tf_buffer.lookup_transform(
                self.p.base_frame, self.p.ee_link, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
        except Exception as e:                                          # noqa: BLE001
            self.get_logger().error(
                f'tcp<-EE calib: TF {self.p.base_frame}->{self.p.ee_link} failed: {e}')
            return False
        tr, rot = tf.transform.translation, tf.transform.rotation
        T_base_tcp = np.eye(4)
        T_base_tcp[:3, :3] = quat_to_mat((rot.x, rot.y, rot.z, rot.w))
        T_base_tcp[:3, 3] = [tr.x, tr.y, tr.z]
        self._T_tcp_ee = np.linalg.inv(T_base_tcp) @ T_base_ee
        self.get_logger().info('calibrated tcp<-EE transform for impedance moves')
        return True

    def _home_tcp_pose(self):
        """Cartesian tcp pose of home_joints via MoveIt /compute_fk (cached). HOME is a joint
        config the impedance controller cannot command directly, so we drive its FK pose."""
        if self._home_tcp is not None:
            return self._home_tcp
        if self._fk_cli is None or not self._fk_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('compute_fk service unavailable (is move_group running?)')
            return None
        req = GetPositionFK.Request()
        req.header.frame_id = self.p.base_frame
        req.fk_link_names = [self.p.ee_link]
        js = JointState()
        js.name = list(self.p.joint_names)
        js.position = [float(x) for x in self.p.home_joints]
        req.robot_state.joint_state = js
        future = self._fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        if resp is None or resp.error_code.val != 1 or not resp.pose_stamped:
            self.get_logger().error('compute_fk failed for the home pose')
            return None
        self._home_tcp = resp.pose_stamped[0]
        return self._home_tcp

    def impedance_move_to_pose(self, tcp_ps, label='', timeout_sec=None):
        """Move the tcp to a Cartesian target using the impedance controller at high stiffness.

        Switches to the impedance controller, ramps the equilibrium pose from the current EE
        pose to the target (speed-limited so the high-stiffness spring does not yank), then
        waits until the measured EE pose converges. Always restores the position controller.

        Input:  tcp_ps -- PoseStamped target for the tcp (ee_link) frame in base_frame.
        Output: (ok, reason).
        """
        log, p = self.get_logger(), self.p
        if self._switch_cli is None:
            return False, 'impedance moves disabled'
        if not self._ensure_tcp_ee_calib():
            return False, 'tcp<-EE calibration failed (FrankaState/TF?)'
        if not self._switch_controllers(activate=[p.impedance_controller_name],
                                        deactivate=[p.position_controller_name]):
            return False, 'switch to impedance failed'
        try:
            self._pub_impedance_stiffness(p.impedance_move_stiffness)
            if self._wait_franka_state(timeout_sec=2.0) is None:
                return False, 'no FrankaState'
            T0 = self._read_otee_T()
            T1 = self._pose_to_T(tcp_ps.pose) @ self._T_tcp_ee
            p0, p1 = T0[:3, 3].copy(), T1[:3, 3].copy()
            q0, q1 = mat_to_quat(T0[:3, :3]), mat_to_quat(T1[:3, :3])
            lin = float(np.linalg.norm(p1 - p0))
            ang = quat_angle(q0, q1)
            dur = max(lin / max(1e-6, p.impedance_move_speed),
                      ang / max(1e-6, p.impedance_move_ang_speed), 0.2)
            period = 1.0 / max(1.0, float(p.insertion_rate_hz))
            nsteps = max(1, int(dur / period))
            log.info(f'IMPEDANCE MOVE [{label}]: {lin*1000:.1f} mm / {math.degrees(ang):.1f} deg '
                     f'over {dur:.1f} s ({nsteps} steps)')
            # speed-limited equilibrium ramp
            for k in range(1, nsteps + 1):
                s = k / nsteps
                ps_pos = (1.0 - s) * p0 + s * p1
                Ts = np.eye(4)
                Ts[:3, :3] = quat_to_mat(quat_slerp(q0, q1, s))
                Ts[:3, 3] = ps_pos
                msg = Float64MultiArray()
                msg.data = self._T_to_cmd(Ts)
                self._imp_pose_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=period)
            # wait for the measured EE to settle on the target
            settle_deadline = self.get_clock().now().nanoseconds + \
                int(p.impedance_move_settle_timeout * 1e9)
            R1 = T1[:3, :3]
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=period)
                Tc = self._read_otee_T()
                if Tc is None:
                    continue
                perr = float(np.linalg.norm(Tc[:3, 3] - p1))
                aerr = math.acos(max(-1.0, min(1.0, (np.trace(Tc[:3, :3].T @ R1) - 1.0) / 2.0)))
                if perr < p.impedance_move_settle_pos and aerr < p.impedance_move_settle_ang:
                    return True, f'reached ({perr*1000:.1f} mm, {math.degrees(aerr):.1f} deg)'
                if self.get_clock().now().nanoseconds > settle_deadline:
                    return False, (f'did not settle (perr={perr*1000:.1f} mm, '
                                   f'aerr={math.degrees(aerr):.1f} deg)')
            return False, 'interrupted'
        finally:
            self._switch_controllers(activate=[p.position_controller_name],
                                     deactivate=[p.impedance_controller_name],
                                     strictness=SwitchController.Request.BEST_EFFORT)

    def goto_pose(self, tcp_ps, label=''):
        """Cartesian move dispatcher: impedance (high stiffness) or MoveIt, per use_impedance_moves."""
        if self.p.use_impedance_moves:
            return self.impedance_move_to_pose(tcp_ps, label)
        ok, code = self.move_to_pose(tcp_ps)
        return ok, f'code {code}'

    def goto_home(self, label='home'):
        """HOME dispatcher: impedance to the FK pose of home_joints, or a MoveIt joint move."""
        if self.p.use_impedance_moves and self.p.impedance_home:
            hp = self._home_tcp_pose()
            if hp is None:
                return False, 'home FK failed'
            return self.impedance_move_to_pose(hp, label)
        ok, code = self.move_to_joints(self.p.joint_names, self.p.home_joints)
        return ok, f'code {code}'

    # ================================================================================
    # planning scene
    # ================================================================================
    def add_ground_plane(self, timeout_sec=5.0):
        """Add a floor collision box so plans avoid going through the ground.

        Adds a flat box whose TOP surface sits at ground_z, pushed into move_group via
        /apply_planning_scene (so RViz's interactive planner sees it too).

        Input:  none (uses ground_z / ground_size / base_frame params).
        Output: True if the scene diff was applied, else False.
        """
        if not self._scene.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('apply_planning_scene unavailable — no ground plane')
            return False
        thickness = 0.02
        co = CollisionObject()
        co.header.frame_id = self.p.base_frame
        co.id = 'floor_collision_box'
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(self.p.ground_size), float(self.p.ground_size), thickness]
        pose = Pose()
        pose.position.z = float(self.p.ground_z) - thickness / 2.0   # top face at ground_z
        pose.orientation.w = 1.0
        co.primitives.append(box)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        future = self._scene.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        resp = future.result()
        ok = bool(resp and resp.success)
        self.get_logger().info(
            f'ground plane top@z={self.p.ground_z:.3f} '
            f'{"added to" if ok else "FAILED to add to"} planning scene')
        return ok

    def remove_ground_plane(self, timeout_sec=5.0):
        """Remove the ground_plane collision object from the planning scene.

        Used while operating intentionally close to the table (e.g. the tip-up grasp/place),
        where the ground plane would otherwise make every plan collide. Restore it afterward
        with add_ground_plane().

        Input:  none.
        Output: True if the scene diff was applied, else False.
        """
        if not self._scene.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('apply_planning_scene unavailable — cannot remove ground plane')
            return False
        co = CollisionObject()
        co.header.frame_id = self.p.base_frame
        co.id = 'ground_plane'
        co.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        future = self._scene.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        resp = future.result()
        ok = bool(resp and resp.success)
        self.get_logger().info(f'ground plane {"removed from" if ok else "FAILED to remove from"} '
                               f'planning scene')
        return ok

    # ================================================================================
    # debug markers (RViz)
    # ================================================================================
    def show_marker(self, name, pose_stamped):
        """Register/update a colored pose arrow for RViz and publish immediately.

        Input:  name -- pose name (sets the color via MARKER_COLORS); pose_stamped.
        Output: none. No-op when debug_markers is disabled.
        """
        if self._marker_pub is None:
            return
        if name not in self._marker_order:
            self._marker_order.append(name)
        self._markers[name] = (pose_stamped, MARKER_COLORS.get(name, (0.8, 0.8, 0.8, 1.0)))
        self._publish_markers()

    def clear_markers(self):
        """Delete all pipeline markers from RViz (wipe stale ones from a previous run).

        Input:  none.
        Output: none. Each marker reappears only when its phase calls show_marker again.
        """
        if self._marker_pub is None:
            return
        self._markers.clear()
        self._marker_order.clear()
        arr = MarkerArray()
        for ns in ('pipeline_poses', 'pipeline_labels'):
            m = Marker()
            m.header.frame_id = self.p.base_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.action = Marker.DELETEALL
            arr.markers.append(m)
        self._marker_pub.publish(arr)

    def _publish_markers(self):
        """Publish all registered poses as arrow + text markers (also called on a timer).

        Input:  none.
        Output: none. Publishes a MarkerArray on /peg_in_hole/debug_markers.
        """
        if self._marker_pub is None or not self._marker_order:
            return
        arr = MarkerArray()
        for idx, name in enumerate(self._marker_order):
            pose, rgba = self._markers[name]
            p, q = pose.pose.position, pose.pose.orientation
            start = np.array([p.x, p.y, p.z], float)
            end = start + rotate_vec((q.x, q.y, q.z, q.w), [0.0, 0.0, 0.08])  # along tool +Z
            arrow = Marker()
            arrow.header.frame_id = self.p.base_frame
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'pipeline_poses'
            arrow.id = idx
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.006, 0.012, 0.02
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = rgba
            arrow.points = [Point(x=float(start[0]), y=float(start[1]), z=float(start[2])),
                            Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))]
            arrow.frame_locked = True
            label = Marker()
            label.header.frame_id = self.p.base_frame
            label.header.stamp = arrow.header.stamp
            label.ns = 'pipeline_labels'
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = p.x, p.y, p.z + 0.03
            label.pose.orientation.w = 1.0
            label.scale.z = 0.02
            label.color.r, label.color.g, label.color.b, label.color.a = rgba
            label.text = name
            label.frame_locked = True
            arr.markers.append(arrow)
            arr.markers.append(label)
        self._marker_pub.publish(arr)

    # ================================================================================
    # THE PIPELINE  -- one sequential pass, read top to bottom
    # ================================================================================
    def run(self):
        """Run the full peg-in-hole sequence once, top to bottom.

        Input:  none (reads params + live perception/TF; drives the real robot).
        Output: True if the peg was grasped and inserted; False on any failure (the robot
                is left in place for inspection and a diagnostic is logged).

        Order: setup -> HOME -> DETECT -> COMPUTE GRASP -> MOVE TO GRASP -> CLOSE GRIPPER
               -> LIFT -> COMPUTE INSERTION -> MOVE TO HOLE -> INSERT -> RETURN HOME -> DONE.
        Object poses are detected BEFORE any grasp pose, marker, or motion is computed.
        """
        log = self.get_logger()

        # --- setup ------------------------------------------------------------------
        self._set_state('SETUP')
        if not self._wait_for_infra():
            return self._abort('SETUP', 'MoveIt/gripper servers unavailable')
        self.clear_markers()                       # wipe stale markers from a previous run
        if self.p.add_ground_plane:
            self.add_ground_plane()

        # --- HOME -------------------------------------------------------------------
        self._set_state('HOME')
        log.info('==> HOME: open gripper, move to ready pose.')
        self.gripper_open()
        ok, code = self.goto_home('HOME')
        if not ok:
            return self._abort('HOME', f'failed to reach HOME ({code})')
        self._dwell(self.p.home_settle_sec)        # let the arm + camera TF settle
        self._pause('HOME', 'DETECT')

        # --- DETECT (must happen before any grasp/marker/motion) --------------------
        self._set_state('DETECT')
        log.info('==> DETECT: read peg + hole poses from FoundationPose.')
        peg = self.get_object_pose(self.p.peg_object)
        hole = self.get_object_pose(self.p.hole_object)
        if peg is None or hole is None:
            return self._abort('DETECT', f'detection incomplete: '
                               f'peg={"ok" if peg else "MISSING"}, '
                               f'hole={"ok" if hole else "MISSING"}')
        log.info(f'PEG  : {fmt_pose(peg)}')
        log.info(f'HOLE : {fmt_pose(hole)}')
        self.show_marker('peg', peg)
        self.show_marker('hole', hole)
        # Is the peg standing up or lying on its side? A lying peg needs a tip-up regrasp first.
        peg_horizontal, peg_tilt_deg, peg_axis = self.classify_peg_orientation(peg)
        if peg_horizontal and self.p.tipup_enabled:
            self._set_state('TIP_UP')
            if not self.tip_up_peg(peg, peg_axis):
                return False                           # _abort already logged
            # Back to HOME, then re-detect the (now upright) peg and continue as usual.
            ok, code = self.goto_home('home after tip-up')
            if not ok:
                return self._abort('TIP-UP', f'home after tip-up failed ({code})')
            self._dwell(self.p.home_settle_sec)
            self._set_state('DETECT')
            log.info('==> RE-DETECT: read peg pose after tip-up.')
            peg = self.get_object_pose(self.p.peg_object)
            if peg is None:
                return self._abort('RE-DETECT', 'peg not found after tip-up')
            log.info(f'PEG (re-detect): {fmt_pose(peg)}')
            self.show_marker('peg', peg)
            still_horizontal, _, _ = self.classify_peg_orientation(peg)
            if still_horizontal:
                return self._abort('TIP-UP', 'peg still horizontal after tip-up '
                                   '(grasp/stand orientation likely needs tuning)')
        self._pause('DETECT', 'COMPUTE GRASP')

        # --- COMPUTE GRASP ----------------------------------------------------------
        self._set_state('COMPUTE_GRASP')
        log.info('==> COMPUTE GRASP.')
        grasp_approach, grasp_pose = self.compute_grasp(peg)
        self.show_marker('grasp_approach', grasp_approach)
        self.show_marker('grasp', grasp_pose)
        self._pause('COMPUTE GRASP', 'MOVE TO GRASP')

        # --- MOVE TO GRASP (approach, then final grasp pose) ------------------------
        self._set_state('MOVE_TO_GRASP')
        log.info('==> MOVE TO GRASP: approach.')
        ok, code = self.goto_pose(grasp_approach, 'grasp approach')
        if not ok:
            return self._abort('MOVE TO GRASP', f'approach motion failed ({code})')
        log.info('==> MOVE TO GRASP: final grasp pose.')
        ok, code = self.goto_pose(grasp_pose, 'grasp')
        if not ok:
            return self._abort('MOVE TO GRASP', f'grasp motion failed ({code})')
        self._pause('MOVE TO GRASP', 'CLOSE GRIPPER')

        # --- CLOSE GRIPPER ----------------------------------------------------------
        self._set_state('CLOSE_GRIPPER')
        log.info('==> CLOSE GRIPPER.')
        held, width, msg = self.gripper_force_grasp()
        wtxt = f'{width:.4f} m' if width is not None else 'unknown'
        log.info(f'gripper: held={held}, width={wtxt}, msg="{msg}"')
        if not held:
            return self._abort('CLOSE GRIPPER', f'no object grasped (width={wtxt})')
        self._pause('CLOSE GRIPPER', 'LIFT')

        # --- LIFT (straight up from the grasp pose) ---------------------------------
        self._set_state('LIFT')
        log.info(f'==> LIFT: straight up +{self.p.lift_height:.3f} m.')
        lift = copy.deepcopy(grasp_pose)
        lift.pose.position.z += self.p.lift_height
        self.show_marker('lift', lift)
        ok, info = self.goto_pose(lift, 'lift')
        if not ok:
            return self._abort('LIFT', f'lift failed ({info})')
        self._pause('LIFT', 'MOVE TO HOLE')

        # --- COMPUTE INSERTION + MOVE TO HOLE ---------------------------------------
        self._set_state('MOVE_TO_HOLE')
        log.info('==> MOVE TO HOLE: approach above the hole.')
        hole_approach, insertion_pose = self.compute_insertion(hole)
        self.show_marker('hole_approach', hole_approach)
        self.show_marker('insertion', insertion_pose)
        ok, code = self.goto_pose(hole_approach, 'hole approach')
        if not ok:
            return self._abort('MOVE TO HOLE', f'move to hole approach failed ({code})')
        self._pause('MOVE TO HOLE', 'INSERT')

        # --- INSERT (force-regulated impedance, or MoveIt Cartesian fallback) -------
        self._set_state('INSERT')
        if self.p.use_impedance_insertion:
            log.info('==> INSERT: force-regulated impedance insertion.')
            ok, reason = self.impedance_insert()
            log.info(f'impedance insert: {"OK" if ok else "FAILED"} -- {reason}')
            if not ok:
                return self._abort('INSERT', reason)
        else:
            log.info('==> INSERT: downward Cartesian insertion.')
            ok, fraction = self.move_cartesian([insertion_pose.pose])
            if not ok:
                return self._abort('INSERT', f'insertion failed (planned fraction {fraction:.2f})')

        # --- RELEASE + RETURN HOME (retract out of the hole) ------------------------
        # impedance_insert() has already handed control back to the position controller, so the
        # whole release/retract/home sequence below runs in POSITION control -- we do NOT
        # re-engage the impedance controller after insertion. Open the gripper FIRST to drop the
        # peg, then move the arm away.
        self._set_state('RETURN_HOME')
        log.info('==> RELEASE: open gripper (position control), then retract and return home.')
        if self.p.open_gripper_at_end:
            self.gripper_open()
        # Unlike the first HOME, this one starts with the tool DOWN in the hole. Going straight
        # to the home joint config from there swings the tool sideways through the fixture (the
        # joint move does not lift out first). Retract to hole_approach -- the known-good free
        # pose above the hole we already reached on the way in -- so the home move starts free.
        # Use the MoveIt position-control moves directly (not the impedance dispatcher) so we
        # stay in position control regardless of use_impedance_moves.
        ok, code = self.move_to_pose(hole_approach)
        if not ok:
            log.warn(f'retract above hole failed (code {code}); attempting return home anyway')
        ok, code = self.move_to_joints(self.p.joint_names, self.p.home_joints)
        log.info(f'return-home motion: {"OK" if ok else "FAILED"} (code {code})')

        # --- DONE (final return to HOME, safe resting pose) -------------------------
        log.info('==================  PEG-IN-HOLE SUCCESS  ==================')
        log.info('Final return to HOME (safe pose).')
        self.move_to_joints(self.p.joint_names, self.p.home_joints)
        self._set_state('DONE')
        return True

    def _abort(self, phase, reason):
        """Log a failure diagnostic and stop the pipeline.

        Input:  phase -- name of the failing phase; reason -- diagnostic string.
        Output: always False (so callers can `return self._abort(...)`).
        """
        log = self.get_logger()
        self._set_state(f'ABORTED:{phase}')
        log.error('==================  PIPELINE ERROR  ==================')
        log.error(f'failed phase : {phase}')
        log.error(f'diagnostic   : {reason}')
        log.error('Aborting. Robot left in place for inspection.')
        return False


def main(args=None):
    """Entry point: init rclpy, run the pipeline once, optionally keep markers alive.

    Input:  args -- passed through to rclpy.init (CLI args).
    Output: none. Spins after the run when keep_alive + debug_markers are set, so RViz
            still shows the computed poses (Ctrl-C to exit).
    """
    rclpy.init(args=args)
    node = PipelineNode()
    try:
        success = node.run()
        node.get_logger().info(f'PIPELINE {"COMPLETED" if success else "ABORTED"}.')
        if node.p.keep_alive and node.p.debug_markers:
            node.get_logger().info('Debug markers stay in RViz — press Ctrl-C to exit.')
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
