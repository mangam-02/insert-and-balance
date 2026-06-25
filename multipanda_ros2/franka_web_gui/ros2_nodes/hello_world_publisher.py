#!/usr/bin/env python3
"""Publishes 'Hello World' to /hello_world at 1 Hz."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')
        self.pub = self.create_publisher(String, '/hello_world', 10)
        self.timer = self.create_timer(1.0, self.publish)
        self.count = 0
        self.get_logger().info('HelloWorldPublisher started')

    def publish(self):
        msg = String()
        msg.data = f'Hello World #{self.count}'
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
