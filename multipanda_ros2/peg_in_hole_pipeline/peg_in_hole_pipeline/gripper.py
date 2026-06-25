"""Gripper control via the franka_gripper action servers (namespace /panda_gripper).

franka_msgs/action/Grasp supports a `force` [N] field, so grasp force IS configurable and
is wired through here. Opening uses Move (no force needed); Homing re-calibrates the gripper.
"""
from franka_msgs.action import Grasp, Homing, Move
import rclpy
from rclpy.action import ActionClient


class GripperClient:
    def __init__(self, node, ns='/panda_gripper'):
        self.node = node
        ns = ns.rstrip('/')
        self._grasp = ActionClient(node, Grasp, f'{ns}/grasp')
        self._move = ActionClient(node, Move, f'{ns}/move')
        self._homing = ActionClient(node, Homing, f'{ns}/homing')

    def wait_for_servers(self, timeout_sec=10.0):
        ok = self._grasp.wait_for_server(timeout_sec=timeout_sec)
        ok &= self._move.wait_for_server(timeout_sec=timeout_sec)
        return ok

    def _run(self, client, goal, timeout_sec):
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=timeout_sec)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, 'goal rejected'
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_sec)
        wrapper = result_future.result()
        if wrapper is None:
            return False, 'no result (timeout)'
        res = wrapper.result
        success = bool(getattr(res, 'success', False))
        return success, getattr(res, 'error', '')

    def grasp(self, width, force, speed=0.1, epsilon_inner=0.005, epsilon_outer=0.005,
              timeout_sec=15.0):
        """Close on an object of ~`width` m with `force` N. Returns (success, msg)."""
        goal = Grasp.Goal()
        goal.width = float(width)
        goal.speed = float(speed)
        goal.force = float(force)
        goal.epsilon.inner = float(epsilon_inner)
        goal.epsilon.outer = float(epsilon_outer)
        self.node.get_logger().info(
            f'GRIPPER grasp: width={width:.4f} m, force={force:.1f} N, speed={speed:.3f} m/s, '
            f'epsilon=({epsilon_inner},{epsilon_outer})')
        return self._run(self._grasp, goal, timeout_sec)

    def open(self, width=0.08, speed=0.1, timeout_sec=15.0):
        goal = Move.Goal()
        goal.width = float(width)
        goal.speed = float(speed)
        self.node.get_logger().info(f'GRIPPER open: width={width:.4f} m, speed={speed:.3f} m/s')
        return self._run(self._move, goal, timeout_sec)

    def homing(self, timeout_sec=20.0):
        if not self._homing.wait_for_server(timeout_sec=5.0):
            return False, 'homing server unavailable'
        self.node.get_logger().info('GRIPPER homing')
        return self._run(self._homing, Homing.Goal(), timeout_sec)
