'''This launch file is taken from the reference of depthimage_to_laserscan package for ros2 
(https://github.com/ros-perception/depthimage_to_laserscan/tree/ros2)'''

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_config = os.path.join(
        get_package_share_directory('depthimage_to_laserscan'), 'config', 'param.yaml')
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            # remappings=[('depth', '/camera/depth/image_rect_raw'),
            #             ('depth_camera_info', '/camera/depth/camera_info')],
            parameters=[param_config])
    ])