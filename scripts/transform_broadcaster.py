#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from nav_msgs.msg import Odometry

'''In order to avoid time related errors and tf errors this tf broadcaster node is created to broadcast the transforms on wall time.
This makes sure that the whole system works on same clock. The node susbcribes the odom topic published by the odometry_wall_timer node 
The transform message field is made to publish wall time.

Usage:
cd ros_ws
source install/setup.bash
ros2 run pod2_description transform_broadcaster.py

First time Errors:

If terminal returns no executable found, it may be to provide permission to execute the file first time.
Steps:
cd ros_ws/src/pod2_description/scripts
chmod +x transform_broadcaster.py
cd ../../..
Try the run command again.

If having ananconda/miniconda installed in the system, try deactivating the environment before ros2 run command.
'''
class TransformBroadcasterNode(Node):
    def __init__(self):
        super().__init__('transform_broadcaster_node')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 1)
        self.get_logger().info('Transform Broadcaster Node started')

    def odom_callback(self, msg):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    transform_broadcaster_node = TransformBroadcasterNode()
    rclpy.spin(transform_broadcaster_node)
    transform_broadcaster_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
