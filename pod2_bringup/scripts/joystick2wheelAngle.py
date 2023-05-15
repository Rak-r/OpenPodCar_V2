#!/usr/bin/env/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import sys
import pdb
from std_msgs.msg import Float64
import subprocess, time
import math

class Joystick_2_wheel(Node):

    def __init__(self):
        super().__init__('joystick2wheel')
        self.pub = self.create_publisher('wheelAngleCmd', Float64, 10)
        self.sub = self.create_subscription('joy', Joy, self.callback_joystick, 1)
        self.get_logger().info("Wheel angle node has been initialized")
    
    def callback_joystick(self, data):
        theta = -(math.pi/4)*data.x
        print(data.x)
        if abs(data.x) < 0.2:
            theta = 0
        msg_out = Float64()
        msg_out.data = theta
        self.pub.publish(msg_out)
def main(args=None):
    rclpy.init(args = args)
    joystick2wheel = Joystick_2_wheel()
    rclpy.spin(joystick2wheel)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
