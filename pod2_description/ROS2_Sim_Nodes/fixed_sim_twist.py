#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion, Twist
import tf2_ros

'''
In order to planning erros when the robot follows the complex global plans, the new Gazebo Ackermann plugin has a bug which at the time of reversing, switches the direction(left/right)
This makes robot to go in oppo. direction, when the plan is correctly generated. The node susbcribes the /cmd_vel topic from either Telop node or NAV2 & publishes to new cmd vel topic 
with handling the correct sign reversal as the original ROS2 cmd_vel message.

Usage:
cd ros_ws
source install/setup.bash
ros2 run pod2_description fixed_sim_twist.py

First time Errors:

If terminal returns no executable found, it may be to provide permission to execute the file first time.
Steps:
cd ros_ws/src/pod2_description/scripts
chmod +x fixed_sim_twist.py
cd ../../..
Try the run command again.

If having ananconda/miniconda installed in the system, try deactivating the environment before ros2 run command.
'''

class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('fixed_sim_twist')


        #Parameters

        self.declare_parameters(
            namespace='',
            parameters=[
                ('chicken_game_experiment', True)
            ])

        #subscriber
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)                                     # either from teleop twist joy or NAV2
        self.chicken_game_sub = self.create_subscription(Twist, '/cmd_vel_chicken_modulated', self.game_cb, 10)         # from the chicken game solution

        #publisher
        self.sim_pub = self.create_publisher(Twist, '/sim_cmd_vel', 10)

        self.cmd_vel = None
        self.chicken_game_twist = None
        
        # Timer to regularly publish the final message
        self.timer = self.create_timer(0.06, self.publish_final_cmd_vel)
        # for time debugging purpose
        self.old_time = self.get_clock().now().nanoseconds / 1e9

    def game_cb(self, msg):

        self.chicken_game_twist = msg
       
    def cmd_vel_cb(self, msg):
        
        self.cmd_vel = msg
       
    def publish_final_cmd_vel(self):
        
        if self.chicken_game_twist:
            final_msg = self.chicken_game_twist
            print('Using chicken game cmd_vel_modulated')
        elif self.cmd_vel:
            final_msg = self.cmd_vel
            print('Using cmd_vel from joy or NAV2')
        else:
            return  # No message to publish

        twist = Twist()
        if final_msg.linear.x < 0.0:
            twist.linear.x = final_msg.linear.x
            twist.angular.z = -final_msg.angular.z
        else:
            twist = final_msg

        self.sim_pub.publish(twist)
   
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
