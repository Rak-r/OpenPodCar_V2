from launch import LaunchDescription
import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rviz_path = PathJoinSubstitution([FindPackageShare("pod2_navigation"), "rviz", "navigation.rviz"])

    nav2_launch = PathJoinSubstitution([FindPackageShare("pod2_bringup", "launch", "bringup.launch.py")])

    default_map_path = PathJoinSubstitution([FindPackageShare("pod2_navigation"), "maps", f'{name_of_map}'.yaml])

    navigation_2_config = PathJoinSubstitution([FindPackageShare("pod2_navigation", "config", "nav2_params.yaml")])

    return LaunchDescription([
        DeclareLaunchArgument(name='sim', default_value='false', description='use sim time'),
        DeclareLaunchArgument(name='rviz', default_value='false', description='run rviz2'),
        DeclareLaunchArgument(name='map', default_value='default_map_path', description='map build by slam toolbox'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_launch), launch_arguments={'map': LaunchConfiguration("map"), 'use_sim_time': LaunchConfiguration("sim"), 'params_file': navigation_2_config}.items()),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen", parameters=[{'use_sim_time': LaunchConfiguration("sim")}])
    ])