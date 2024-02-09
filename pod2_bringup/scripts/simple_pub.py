#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Wrench, Twist
from sensor_msgs.msg import JointState


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        # self.publisher_ = self.create_publisher(Wrench, '/force_test', 10)
        # self.publisher2_ = self.create_publisher(JointState, '/joint_states', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.twist_callback)
        self.i = 0

    # def timer_callback(self):
    #     msg = Wrench()
    #     msg.force.x = 0.0
    #     msg.force.y = 0.0
    #     msg.force.z = 0.0
    #     msg.torque.z = 0.0
    #     #msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     #self.get_logger().info('Publishing: "%s"' % msg.data)
    #     #self.i += 1
    # def joint_callback(self):
    #     msg2 = JointState()
    #     msg2.name = "trackingrod_right_pivot_joint"
    #     msg2.position =-1.5
    #     self.publisher2_.publish(msg2)
    def twist_callback(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()