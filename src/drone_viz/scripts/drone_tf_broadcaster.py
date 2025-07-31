#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class DroneTFBroadcaster(Node):
    def __init__(self):
        super().__init__('drone_tf_broadcaster')

        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/drone10/pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"Received drone10 pose: {msg.pose.position.x}, "
            f"{msg.pose.position.y}, {msg.pose.position.z}"
        )

        t = TransformStamped()
        t.header.stamp = msg.header.stamp  # use timestamp from PoseStamped
        t.header.frame_id = 'arena'  # e.g., "map" or "world"
        t.child_frame_id = 'base_link'  # must match the link in URDF

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DroneTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
