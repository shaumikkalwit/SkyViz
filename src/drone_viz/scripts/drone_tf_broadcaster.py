#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped, TransformStamped
# from tf2_ros import TransformBroadcaster

# class DroneTFBroadcaster(Node):
#     def __init__(self):
#         super().__init__('drone_tf_broadcaster')

#         self.br = TransformBroadcaster(self)
#         self.subscription = self.create_subscription(
#             PoseStamped,
#             '/drone10/pose',
#             self.pose_callback,
#             10
#         )

#     def pose_callback(self, msg: PoseStamped):
#         self.get_logger().info(
#             f"Received drone10 pose: {msg.pose.position.x}, "
#             f"{msg.pose.position.y}, {msg.pose.position.z}"
#         )

#         t = TransformStamped()
#         t.header.stamp = msg.header.stamp  # use timestamp from PoseStamped
#         t.header.frame_id = 'arena'  # e.g., "map" or "world"
#         t.child_frame_id = 'base_link'  # must match the link in URDF

#         t.transform.translation.x = msg.pose.position.x
#         t.transform.translation.y = msg.pose.position.y
#         t.transform.translation.z = msg.pose.position.z
#         t.transform.rotation = msg.pose.orientation

#         self.br.sendTransform(t)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DroneTFBroadcaster()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class DroneTFBroadcaster(Node):
    def __init__(self):
        super().__init__('drone_tf_broadcaster')

        self.declare_parameter('drone_name', 'droneXX')
        self.declare_parameter('pose_topic', '/droneXX/pose')

        self.drone_name = self.get_parameter('drone_name').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value

        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

    def pose_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"Received pose for {self.drone_name}: "
            f"{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}"
        )
        
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'arena'
        t.child_frame_id = f'{self.drone_name}_base_link'

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
