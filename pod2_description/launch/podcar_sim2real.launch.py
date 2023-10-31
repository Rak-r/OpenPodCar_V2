import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    odometry_node = Node(
        package='pod2_description',
        name='odometry_publisher',
        executable= 'odometry_wall_time.py',
        output = 'screen'
    )

    laser_scan_node = Node(
        package='pod2_description',
        name='scan_publisher_node',
        executable='laser_sim2real.py',
        output = 'screen'
    )
    camera_node = Node(
        package='pod2_description',
        name='camera_node',
        executable='testing_tf.py',
        output = 'screen'
    )
    
    tf_broadcaster = Node(
        package='pod2_description',
        name='transform_broadcaster',
        executable='transform_broadcaster.py',
        output= 'screen'
    )

    ld.add_action(odometry_node)
    ld.add_action(laser_scan_node)
    ld.add_action(tf_broadcaster)
    ld.add_action(camera_node)

    return ld