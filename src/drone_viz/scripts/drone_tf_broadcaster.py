#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import yaml
import os

class DroneTFBroadcaster(Node):
    def __init__(self):
        super().__init__('drone_tf_broadcaster')
        self.declare_parameter('config_file', '')

        config_path = self.get_parameter('config_file').get_parameter_value().string_value
        if not os.path.exists(config_path):
            self.get_logger().error(f"Config file does not exist: {config_path}")
            return

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.br = TransformBroadcaster(self)
        self.subscribers = []

        for drone in config['drones']:
            drone_id = drone['id']
            topic = drone['topic']
            self.get_logger().info(f"Subscribing to {topic} for {drone_id}")
            sub = self.create_subscription(
                PoseStamped,
                topic,
                self.make_pose_callback(drone_id),
                10
            )
            self.subscribers.append(sub)

    def make_pose_callback(self, drone_id):
        def callback(msg):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'  # or 'map', depending on your setup
            t.child_frame_id = drone_id
            t.transform.translation = msg.pose.position
            t.transform.rotation = msg.pose.orientation
            self.br.sendTransform(t)
        return callback

def main(args=None):
    rclpy.init(args=args)
    node = DroneTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()
