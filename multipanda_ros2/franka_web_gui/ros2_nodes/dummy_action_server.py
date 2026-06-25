#!/usr/bin/env python3
"""Dummy action server — simulates 0→100% progress for Pick/Place/Wave/Handshake."""
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from action_msgs.msg import GoalStatus

# Uses the standard FollowJointTrajectory action as a stand-in.
# Replace with custom franka_web_gui action types once defined.
from control_msgs.action import FollowJointTrajectory


class DummyActionServer(Node):
    def __init__(self):
        super().__init__('dummy_action_server')
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            '/franka/execute_skill',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        self.get_logger().info('DummyActionServer ready on /franka/execute_skill')

    def _goal_cb(self, _goal):
        self.get_logger().info('[DUMMY] Action goal received → ACCEPTED')
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal):
        self.get_logger().info('[DUMMY] Cancel requested → ACCEPTED')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        self.get_logger().info('[DUMMY] Action executing…')
        feedback = FollowJointTrajectory.Feedback()

        for progress in range(0, 101, 5):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('[DUMMY] Action cancelled')
                return FollowJointTrajectory.Result()

            self.get_logger().info(f'[DUMMY] Progress: {progress}%')
            goal_handle.publish_feedback(feedback)
            time.sleep(0.2)

        goal_handle.succeed()
        self.get_logger().info('[DUMMY] Action succeeded')
        return FollowJointTrajectory.Result()


def main(args=None):
    rclpy.init(args=args)
    node = DummyActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
