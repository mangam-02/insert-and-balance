#!/usr/bin/env python3
"""Subscribes to /hello_world and /hello_world_command and logs messages."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldSubscriber(Node):
    def __init__(self):
        super().__init__('hello_world_subscriber')
        self.create_subscription(String, '/hello_world', self.on_hello, 10)
        self.create_subscription(String, '/hello_world_command', self.on_command, 10)
        self.get_logger().info('HelloWorldSubscriber started — listening on /hello_world and /hello_world_command')

    def on_hello(self, msg: String):
        self.get_logger().info(f'[/hello_world] {msg.data}')

    def on_command(self, msg: String):
        self.get_logger().info(f'[/hello_world_command] Received command: "{msg.data}"')
        if msg.data == 'start':
            self.get_logger().info('→ START command received')
        elif msg.data == 'stop':
            self.get_logger().info('→ STOP command received')
        elif msg.data == 'reset':
            self.get_logger().info('→ RESET command received')


def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
