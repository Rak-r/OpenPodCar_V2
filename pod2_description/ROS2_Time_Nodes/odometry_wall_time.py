#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

'''In order to avoid time related errors and tf errors this node is created to convert the messages coming from gazebo to wall time. 
This makes sure that the whole system works on same clock. The node susbcribes the ground truth odometry topic from GZ and publishes to ROS2 topic /odom
with changing the time stamp to wall time.

Usage:
cd ros_ws
source install/setup.bash
ros2 run pod2_description odometry_wall_timer.py

First time Errors:

If terminal returns no executable found, it may be to provide permission to execute the file first time.
Steps:
cd ros_ws/src/pod2_description/scripts
chmod +x odometry_wall_timer.py
cd ../../..
Try the run command again.

If having ananconda/miniconda installed in the system, try deactivating the environment before ros2 run command.
'''

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.sub = self.create_subscription(Odometry, '/model/podcar/odometry', self.callback, 10)
        self.get_logger().info('Odometry wall timer node is started')
        
    def callback(self, msg):
        self.odometry_msg = msg
        self.odometry_msg.header.frame_id = 'odom'
        self.odometry_msg.child_frame_id = 'base_link'
        self.odometry_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.odometry_msg)
            

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
