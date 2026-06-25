"""Motion execution against the running MoveIt /move_group, with no moveit_py/pymoveit2
dependency (mirrors franka_camera_calibration.move_group_client, extended with joint-space
goals and straight-line Cartesian motion for the insertion).

  * move_to_pose      -> free-space plan+execute to a Cartesian goal (MoveGroup action)
  * move_to_joints    -> free-space plan+execute to a joint configuration (e.g. HOME)
  * move_cartesian    -> straight-line path through waypoints (GetCartesianPath + ExecuteTrajectory)

TODO: swap the straight-line insertion for force/impedance-controlled insertion once a
compliant controller is available (the free-space + Cartesian planners here are position-only).
"""
from geometry_msgs.msg import PoseStamped, Vector3
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume, Constraints, JointConstraint, OrientationConstraint,
    PositionConstraint, WorkspaceParameters,
)
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive
import rclpy
from rclpy.action import ActionClient


class MotionClient:
    def __init__(self, node, *, group_name, base_frame, ee_link,
                 position_tolerance=0.005, orientation_tolerance=0.01,
                 planning_time=10.0, planning_attempts=10,
                 vel_scale=0.1, acc_scale=0.1):
        self.node = node
        self.group_name = group_name
        self.base_frame = base_frame
        self.ee_link = ee_link
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.planning_time = planning_time
        self.planning_attempts = planning_attempts
        self.vel_scale = vel_scale
        self.acc_scale = acc_scale
        self._move = ActionClient(node, MoveGroup, 'move_action')  # MoveIt2 advertises /move_action
        self._exec = ActionClient(node, ExecuteTrajectory, 'execute_trajectory')
        self._cart = node.create_client(GetCartesianPath, 'compute_cartesian_path')

    def wait_for_servers(self, timeout_sec=15.0):
        ok = self._move.wait_for_server(timeout_sec=timeout_sec)
        ok &= self._exec.wait_for_server(timeout_sec=timeout_sec)
        ok &= self._cart.wait_for_service(timeout_sec=timeout_sec)
        return ok

    # --- shared request scaffolding -----------------------------------------------
    def _base_request(self, req):
        req.group_name = self.group_name
        req.num_planning_attempts = self.planning_attempts
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        ws = WorkspaceParameters()
        ws.header.frame_id = self.base_frame
        ws.min_corner = Vector3(x=-1.5, y=-1.5, z=-1.5)
        ws.max_corner = Vector3(x=1.5, y=1.5, z=1.5)
        req.workspace_parameters = ws

    def _send_move_goal(self, goal, timeout_sec):
        send = self._move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=timeout_sec)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_sec)
        wrapper = result_future.result()
        if wrapper is None:
            return False, None
        code = wrapper.result.error_code.val          # MoveItErrorCodes.SUCCESS == 1
        return code == 1, code

    # --- Cartesian pose goal ------------------------------------------------------
    def move_to_pose(self, pose: PoseStamped, timeout_sec=60.0):
        goal = MoveGroup.Goal()
        self._base_request(goal.request)
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.ee_link
        pc.weight = 1.0
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[self.position_tolerance])
        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(pose.pose)
        pc.constraint_region = region
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = self.orientation_tolerance
        oc.absolute_y_axis_tolerance = self.orientation_tolerance
        oc.absolute_z_axis_tolerance = self.orientation_tolerance
        oc.weight = 1.0
        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        goal.request.goal_constraints.append(c)
        goal.planning_options.plan_only = False
        return self._send_move_goal(goal, timeout_sec)

    # --- joint-space goal (HOME) --------------------------------------------------
    def move_to_joints(self, joint_names, positions, tolerance=0.01, timeout_sec=60.0):
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

    # --- straight-line Cartesian path (insertion) ---------------------------------
    def move_cartesian(self, waypoints, eef_step=0.005, min_fraction=0.9, timeout_sec=60.0):
        """Plan a straight-line path through `waypoints` (geometry_msgs/Pose in base_frame)
        and execute it. Returns (ok, fraction)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.group_name = self.group_name
        req.link_name = self.ee_link
        req.waypoints = list(waypoints)
        req.max_step = float(eef_step)
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        future = self._cart.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        resp = future.result()
        if resp is None:
            return False, 0.0
        fraction = resp.fraction
        if resp.error_code.val != 1 or fraction < min_fraction:
            return False, fraction
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = resp.solution
        send = self._exec.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=timeout_sec)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, fraction
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_sec)
        wrapper = result_future.result()
        ok = wrapper is not None and wrapper.result.error_code.val == 1
        return ok, fraction
