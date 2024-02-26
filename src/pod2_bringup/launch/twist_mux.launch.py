import rclpy
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    mux_topic_params = PathJoinSubstitution([FindPackageShare("pod2_bringup"), "config", "twist_mux_topics.yaml"])
    mux_lock_params =  PathJoinSubstitution([FindPackageShare("pod2_bringup"), "config", "twist_mux_locks.yaml"])

    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Use simulation time'
        ),
        

         Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[mux_lock_params,
                        mux_topic_params],
            remappings=[('cmd_vel', '/cmd_vel_out')]
        )
        ])