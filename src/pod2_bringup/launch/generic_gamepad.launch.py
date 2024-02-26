import rclpy
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.conditions import IfCondition
import os
from ament_index_python import get_package_share_directory
'''This launch file uses the ros2 teleop-twist-joy pakcage node 
and used to control the robot in simulation as well as the physical with joystick controller'''

def generate_launch_description():

    # specify the path for he custom defined joy stick parameters according to the joystick available
    joy_params = PathJoinSubstitution([FindPackageShare("pod2_bringup"), "config", "generic_gamepad.yaml"])

    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Use simulation time'
        ),
        Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'deadzone': 0.01,
                'autorepeat_rate': 20.0,
                'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

         Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_params],
            #condition = IfCondition(LaunchConfiguration("teleop_node")),
            # remappings=[('cmd_vel', '/model/podcar/cmd_vel')]
        )

        
    ])

