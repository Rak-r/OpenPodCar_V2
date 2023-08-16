import rclpy
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    joy_params = PathJoinSubstitution([FindPackageShare("pod2_bringup"), "config", "joy.yaml"])

    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation time'
        ),
        Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'deadzone': 0.3,
                'autorepeat_rate': 20.0,
                'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

         Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_params],
            remappings=[('cmd_vel', '/model/podcar/cmd_vel')]
        )

        # Node(
        #     package='teleop_twist_joy',
        #     executable='teleop_node',
        #     name='teleop_twist_joy_node',
        #     output ='screen',
        #     parameters=[{'enable_button':5 },
        #         {'require_enable_button': True},
        #         {'enable_turbo_button': 4},
        #         {'axis_linear.x': 1},
        #         {'axis_angular.yaw': 2},
        #         {'scale_linear.x': 1.0},
        #         {'scale_angular.yaw': 1.0},
        #                 {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        #     remappings=[('/cmd_vel', '/model/podcar/cmd_vel')]
        # )
    ])

