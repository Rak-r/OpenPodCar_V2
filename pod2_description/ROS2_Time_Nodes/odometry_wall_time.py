#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import tf2_ros

'''In order to avoid time related errors and tf errors this node is created to convert the messages coming from gazebo to wall time. 
This makes sure that the whole system works on same clock. The node susbcribes the ground truth odometry topic from GZ and publishes to ROS2 topic /odom
with changing the time stamp to wall time and it also publishes the transform between odom and base_link.

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
        self.old_time = self.get_clock().now().nanoseconds / 1e9
        self.br = tf2_ros.TransformBroadcaster(self)
    def callback(self, msg):

        odom_msg = Odometry()
        transform = TransformStamped()
        timestamp = self.get_clock().now().nanoseconds / 1e9
        callback_period = timestamp - self.old_time
        self.old_time = timestamp
        print('timestamp', timestamp)
        print('old', self.old_time)
        print('callback_period', callback_period)
        print('**************')
        
        try:
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = msg.header.frame_id
            odom_msg.child_frame_id = msg.child_frame_id
            self.get_logger().info('publishing the odometry message')
            self.pub.publish(odom_msg)

            transform.header = odom_msg.header
            transform.child_frame_id = odom_msg.child_frame_id
            # transform.transform.translation = Vector3(x=odom_msg.pose.pose.position.x, y=odom_msg.pose.pose.position.y,z=odom_msg.pose.pose.position.z)
            transform.transform.translation.x = msg.pose.pose.position.x
            transform.transform.translation.y = msg.pose.pose.position.y
            transform.transform.translation.z = msg.pose.pose.position.z
            transform.transform.rotation.x = msg.pose.pose.orientation.x
            transform.transform.rotation.y = msg.pose.pose.orientation.y
            transform.transform.rotation.z = msg.pose.pose.orientation.z
            transform.transform.rotation.w = msg.pose.pose.orientation.w
            self.br.sendTransform(transform)
            self.get_logger().info('Sending the transform')
        
        except ValueError as e:
            self.get_logger().info('No incoming odometry messages', {e})
            
            exit()

            
def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
