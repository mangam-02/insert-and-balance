"""Minimal MoveGroup action client.

Connects to the ``/move_group`` action server started by
``franka_moveit_config moveit.launch.py`` and plans+executes a motion to a
single Cartesian goal pose, expressed for a given end-effector link in a given
planning frame.  Keeping this self-contained avoids depending on pymoveit2 /
moveit_py, which may or may not be present in the container.
"""

from geometry_msgs.msg import PoseStamped, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    OrientationConstraint,
    PositionConstraint,
    WorkspaceParameters,
)
from shape_msgs.msg import SolidPrimitive
import rclpy
from rclpy.action import ActionClient


class MoveGroupClient:
    def __init__(self, node, *, group_name, base_frame, ee_link,
                 position_tolerance=0.005, orientation_tolerance=0.01,
                 planning_time=10.0, planning_attempts=10,
                 vel_scale=0.1, acc_scale=0.1, action_name='move_group'):
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
        self._client = ActionClient(node, MoveGroup, action_name)

    def wait_for_server(self, timeout_sec=15.0):
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def _build_goal(self, pose: PoseStamped) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        req = goal.request

        req.group_name = self.group_name
        req.num_planning_attempts = self.planning_attempts
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale

        ws = WorkspaceParameters()
        ws.header.frame_id = self.base_frame
        ws.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        ws.max_corner = Vector3(x=1.0, y=1.0, z=1.0)
        req.workspace_parameters = ws

        # Position constraint: small sphere around the target point.
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.ee_link
        pc.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance]
        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(pose.pose)
        pc.constraint_region = region

        # Orientation constraint.
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = self.orientation_tolerance
        oc.absolute_y_axis_tolerance = self.orientation_tolerance
        oc.absolute_z_axis_tolerance = self.orientation_tolerance
        oc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        req.goal_constraints.append(constraints)

        # plan_only = False -> move_group plans AND executes.
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        return goal

    def move_to_pose(self, pose: PoseStamped, timeout_sec=60.0):
        """Plan + execute to the pose. Returns (ok, error_code)."""
        goal = self._build_goal(pose)

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_sec)
        result_wrapper = result_future.result()
        if result_wrapper is None:
            return False, None

        error_code = result_wrapper.result.error_code.val
        # moveit_msgs/MoveItErrorCodes.SUCCESS == 1
        return error_code == 1, error_code
