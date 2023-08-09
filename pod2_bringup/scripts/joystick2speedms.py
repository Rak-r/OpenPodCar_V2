#!/usr/bin/env python3
import sys
import rclpy
import pdb
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import subprocess, time
from rclpy.node import Node
from std_msgs.msg import String

def rescale(x, oldmin, oldmax, newmin, newmax):
    r = (x-oldmin)/(oldmax-oldmin)
    out = newmin + r*(newmax-newmin)
    return out

class Joystick2speed(Node):

    def __init__(self):
        super().__init__('joystick2speed')
        self.pub = self.create_publisher(Float64,'speedcmd_metersec', 1)
        self.sub = self.create_subscription( Joy, '/joy', self.joy_callback, 1)
        self.buffer = []
        for i in range(0,100):
            self.buffer.append('X')
            self.buffer_index = 0
            self.get_logger().info("Speed node has been initialized")
    def joy_callback(self, data):
        joystick_y = -data.axes[0]
        #print(joystick_y)
        deadmanzone_min = -0.2
        deadmanzone_max = 0.2
        max_fwd_speed_per_second = 2
        max_bkwd_speed_per_sec = 2
        if joystick_y > deadmanzone_min and joystick_y < deadmanzone_max:
            velocity = 0
        elif joystick_y > 0:
            velocity = rescale(joystick_y, deadmanzone_max, 1., 0., max_fwd_speed_per_second)
        elif joystick_y < 0:
            velocity = rescale(joystick_y, -1., deadmanzone_min, -max_bkwd_speed_per_sec, 0.)
        msg_out = Float64()
        msg_out.data

        print(velocity)
        self.pub.publish(msg_out)
def main(args = None):
    rclpy.init(args=args)
    joystick2speed = Joystick2speed()
    rclpy.spin(joystick2speed)
    rclpy.shutdown()  
if __name__ == '__main__':
    main()
