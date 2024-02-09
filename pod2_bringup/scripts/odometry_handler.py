#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

class odom_handler(Node):
    def __init__(self):
        super().__init__('odometry_handler')

        self.odomPub = self.create_publisher(Odometry, 'nav_msgs/Odometry', self.callback, 1)
        self.odomSub = self.create_subscription(Odometry, 'odometry/groundTruth', self.callback, 1)

    def callback(self, msg):
        self.odomPub.publish(msg)

def main(args = None):
    rclpy.init()
    odometry_handler = odom_handler()
    try:
        rclpy.spin(odometry_handler)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    
    
