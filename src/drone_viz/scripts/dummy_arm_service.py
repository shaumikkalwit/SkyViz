#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class DummyArmService(Node):
    def __init__(self):
        super().__init__('dummy_arm_service')
        self.srv = self.create_service(SetBool, '/arm', self.callback)
        self.get_logger().info('Dummy /arm service is ready.')

    def callback(self, request, response):
        if request.data:
            self.get_logger().info('Received arm request.')
            response.success = True
            response.message = 'Armed successfully!'
        else:
            response.success = False
            response.message = 'Request was false.'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DummyArmService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
