from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
import os

def generate_launch_description():
    param_config = os.path.join(
        get_package_share_directory('pod2_sensor_tools'), 'config', 'param.yaml')
    
    
    return LaunchDescription([

        DeclareLaunchArgument(
            name='depth',
            default_value='false',
            description= 'Whether to run depth_to_laserscan in case of using depth camera'
        ),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            # remappings=[('depth', '/depth/image_rect_raw'),
            #             ('depth_camera_info', '/camera/depth/camera_info')],
            parameters=[param_config],
            condition = IfCondition(LaunchConfiguration('depth'))),
        
        Node(
            package='ros2_laser_scan_matcher',
            executable= 'laser_scan_matcher',
            name='laser_scan_matcher',
            output='screen'
        )
    ])