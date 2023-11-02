#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo

'''In order to avoid time related errors and tf errors this node is created to convert the messages coming from gazebo to wall time. 
This makes sure that the whole system works on same clock. The node susbcribes the depth, rgb image and camera info topic from GZ camera plugin and publishes to ROS2 topic /scan
with changing the time stamp to wall time.

Usage:
cd ros_ws
source install/setup.bash
ros2 run pod2_description Depth_image_2_real.py

First time Errors:

If terminal returns no executable found, it may be to provide permission to execute the file first time.
Steps:
cd ros_ws/src/pod2_description/ROS2_Time_Nodes
chmod +x Depth_image_2_real.py
cd ../../..
Try the run command again.

If having ananconda/miniconda installed in the system, try deactivating the environment before ros2 run command.
'''
class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.rgb_subscription = self.create_subscription(Image, '/rgbd_camera/image', self.image_callback, 1)          # subscribe to the rgb image topic
        self.depth_subscription = self.create_subscription(Image, '/rgbd_camera/depth_image', self.depth_callback, 1)  # subscribe to depth image topic
        self.info_sub = self.create_subscription(CameraInfo, 'rgbd_camera/camera_info', self.camera_info_callback, 1)  # subscribe to camera info topic
        self.rgb_pub = self.create_publisher(Image, '/rgbd_image_real', 1)
        self.depth_publisher = self.create_publisher(Image, '/depth', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/depth_camera_info', 1)
        self.get_logger().info('Camera Publisher Node started')

    def depth_callback(self, msg):
        # Convert to ROS2 timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_publisher.publish(msg)
    def image_callback(self, msg):
        # Publish the received scan data with the current ROS 2 time stamp
        msg.header.stamp = self.get_clock().now().to_msg()
        self.rgb_pub.publish(msg)
    
    def camera_info_callback(self, msg):
        # Publish the camera info on ROS2 time
        msg.header.stamp = self.get_clock().now().to_msg()
        self.info_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_node = ImagePublisherNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()