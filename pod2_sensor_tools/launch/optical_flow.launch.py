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

    optical_flow_sensor_node = Node(
        package='pod2_sensor_tools',
        name='optical_flow_publisher',
        executable= 'Optical_Flow_Odom.py',
        output = 'screen'
    )

    ld.add_action( optical_flow_sensor_node )
    

    return ld