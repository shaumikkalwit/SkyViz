#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

class DroneTFBroadcaster(Node):
    def __init__(self):
        super().__init__('drone_tf_broadcaster')

        self.declare_parameter('drone_name', 'droneXX')
        self.declare_parameter('pose_topic', '/droneXX/pose')
        self.declare_parameter('mesh', 'crazyflie.stl')  # default fallback

        self.drone_name = self.get_parameter('drone_name').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.mesh = self.get_parameter('mesh').get_parameter_value().string_value

        self.br = TransformBroadcaster(self)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        self.drone_subscriptions = self.create_subscription(
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

        # Publish mesh marker
        marker = Marker()
        marker.header.frame_id = 'arena'
        marker.header.stamp = msg.header.stamp
        marker.ns = self.drone_name
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose = msg.pose
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.mesh_resource = f"package://drone_viz/meshes/{self.mesh}"
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = DroneTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()