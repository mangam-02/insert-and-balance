#!/usr/bin/env python3
"""Publishes mock robot status and fake joint states at 10 Hz."""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
    'panda_joint5', 'panda_joint6', 'panda_joint7',
]


class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.1, self.publish_joints)
        self.create_timer(1.0, self.publish_status)
        self.t = 0.0
        self.get_logger().info('RobotStatusPublisher started')

    def publish_status(self):
        msg = String()
        msg.data = 'IDLE'
        self.status_pub.publish(msg)

    def publish_joints(self):
        self.t += 0.1
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [math.sin(self.t + i * 0.5) * 0.8 for i in range(7)]
        msg.velocity = [math.cos(self.t + i * 0.5) * 0.1 for i in range(7)]
        msg.effort = [v * 2.0 for v in msg.velocity]
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
