#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import time
import math
import tf_transformations as tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class Odom_node(Node):
    def __init__(self):
        super().__init__('odom_node')
        
        self.speed_sub = self.create_subscription(Float64, 'speedmeter_sec', self.callback_speed, 1)
        self.wheelangle_sub = self.create_subscription(Float64, 'wheelAngleCmd', self.callback_wheelangle, 1)
        self.current_time = self.get_clock().now().to_msg()
        self.last_time = time.time()
        self.x = 0.
        self.y = 0.
        self.th = 0.
        self.vx = 0.
        self.vth = 0.
        self.dt = 0.0
        self.wheelbase = 1.05
        self.current_angle = 0.
        

    def callback_speed(self, msg):
        self.vx= msg.data/3.5
    
    def callback_wheelangle(self, msg):
        self.th = -msg.data/4
    
def main(args=None):
    rclpy.init(args=args)
    node = Odom_node()
    odom_pub = node.create_publisher(Odometry, 'odom/groundtruth', 1)
    try:
        while rclpy.ok():
            current_time = time.time()
            #last_time = node.get_clock().now().to_msg()
            node.dt = (current_time - node.last_time)
            node.vth = node.vx*math.tan(node.current_angle)/node.wheelbase
            print("current angle"+ str(node.current_angle)+ "\n")
            delta_th = node.vth*node.dt
            print("Delta_th: " + str(delta_th) + "\n")

            delta_x = (node.vx * math.cos(node.th)) * node.dt
            delta_y = (node.vx * math.sin(node.th)) * node.dt

            node.x += delta_x
            node.y += delta_y
            node.th += delta_th
            print("Theta: " + str(node.th) + "\n")

            odom_quat = tf.quaternion_from_euler(0.0, 0.0, node.th/2)
            #print(odom_quat)
            #odom_quat_two = tf.eu
            odom = Odometry()
            odom.header.stamp = node.current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = node.x
            odom.pose.pose.position.y =  node.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = odom_quat[0]
            odom.pose.pose.orientation.y = odom_quat[1]
            odom.pose.pose.orientation.z = odom_quat[2]
            odom.pose.pose.orientation.w = odom_quat[3]

        
            odom_pub.publish(odom)
            node.last_time = current_time
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()