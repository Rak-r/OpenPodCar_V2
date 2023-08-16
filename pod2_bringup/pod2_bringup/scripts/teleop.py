#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('cmd_publisher')
    publisher = node.create_publisher(Twist, '/model/podcar/cmd_vel', 10)
    sub = node.create_subscription(Joy, '/joy', initial)

    def initial(data):
        move = Twist()
        move.linear.x = 1.5*data.axes[1]
        move.angular.z = 1.5*data.axes[2]
        publisher.publish(move)
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()