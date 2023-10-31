#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

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
class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.subscription = self.create_subscription(
            Image, '/rgbd_image/image', self.image_callback, 1)
        self.publisher_ = self.create_publisher(Image, '/rgbd_image_real', 1)
        self.get_logger().info('Camera Publisher Node started')

    def image_callback(self, msg):
        # Publish the received scan data with the current ROS 2 time stamp
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_node = ImagePublisherNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()