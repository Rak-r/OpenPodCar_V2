#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import sys
import pdb
from std_msgs.msg import Float64
import subprocess, time
import math

class Joystick_to_wheel(Node):

    def __init__(self):
        super().__init__('joystick_to_wheel')
        self.pub = self.create_publisher(Float64,'/wheelAngleCmd', 10)
        self.sub = self.create_subscription(Joy, '/joy', self.callback_joystick, 1)
        self.get_logger().info("Wheel angle node has been initialized")
    
    def callback_joystick(self, data):
        theta = -(math.pi/4)*data
        print(data)
        if abs(data) < 0.2:
            theta = 0
        msg_out = Float64()
        msg_out.data = theta
        self.pub.publish(msg_out)
def main(args=None):
    rclpy.init(args = args)
    joystick_to_wheel = Joystick_to_wheel()
    rclpy.spin(joystick_to_wheel)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
