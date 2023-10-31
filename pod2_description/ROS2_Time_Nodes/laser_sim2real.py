#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

'''In order to avoid time related errors and tf errors this node is created to convert the messages coming from gazebo to wall time. 
This makes sure that the whole system works on same clock. The node susbcribes the lidar scan topic from GZ laser plugin and publishes to ROS2 topic /scan
with changing the time stamp to wall time.

Usage:
cd ros_ws
source install/setup.bash
ros2 run pod2_description laser_sim2real.py

First time Errors:

If terminal returns no executable found, it may be to provide permission to execute the file first time.
Steps:
cd ros_ws/src/pod2_description/scripts
chmod +x laser_sim2real.py
cd ../../..
Try the run command again.

If having ananconda/miniconda installed in the system, try deactivating the environment before ros2 run command.
'''
class ScanPublisherNode(Node):
    def __init__(self):
        super().__init__('scan_publisher_node')
        self.subscription = self.create_subscription(
            LaserScan, '/lidar_scan', self.scan_callback, 1)
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 1)
        self.get_logger().info('Scan Publisher Node started')

    def scan_callback(self, msg):
        # Publish the received scan data with the current ROS 2 time stamp
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    scan_publisher_node = ScanPublisherNode()
    rclpy.spin(scan_publisher_node)
    scan_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()