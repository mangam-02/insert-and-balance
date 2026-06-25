#!/usr/bin/env python3
"""Dummy service server for /robot/start, /robot/stop, /robot/home."""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class DummyServiceServer(Node):
    def __init__(self):
        super().__init__('dummy_service_server')
        self.create_service(Trigger, '/robot/start', self._handle_start)
        self.create_service(Trigger, '/robot/stop', self._handle_stop)
        self.create_service(Trigger, '/robot/home', self._handle_home)
        self.get_logger().info('DummyServiceServer ready on /robot/start, /robot/stop, /robot/home')

    def _handle_start(self, _req, response):
        self.get_logger().info('[DUMMY] /robot/start called → success')
        response.success = True
        response.message = 'Robot started (DUMMY)'
        return response

    def _handle_stop(self, _req, response):
        self.get_logger().info('[DUMMY] /robot/stop called → success')
        response.success = True
        response.message = 'Robot stopped (DUMMY)'
        return response

    def _handle_home(self, _req, response):
        self.get_logger().info('[DUMMY] /robot/home called → success')
        response.success = True
        response.message = 'Robot moved to home (DUMMY)'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DummyServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
