#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class WallClockPublisherNode(Node):
    def __init__(self):
        super().__init__('wall_clock_publisher_node')
        self.publisher_ = self.create_publisher(Clock, '/clock', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Wall Clock Publisher Node started')

    def timer_callback(self):
        msg = Clock()
        current_time = rclpy.clock.Clock().now().to_msg()
        msg.clock = current_time
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing wall clock message')

def main(args=None):
    rclpy.init(args=args)
    node = WallClockPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

