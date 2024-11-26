from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_path, get_package_share_directory
import os

def generate_launch_description():
    

    # Define a boolean launch argument
    realsense_enable_arg = DeclareLaunchArgument(
        'realsense_enable',
        default_value='true',
        description='Enable Realsense camera node'
    )

    intel_camera_node_path = get_package_share_directory('realsense2_camera')
    
    realsense_camera_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(intel_camera_node_path, 'launch', 'rs_launch.py')),condition=IfCondition(LaunchConfiguration('realsense_enable')))
    
    return LaunchDescription([
        

        realsense_camera_launch,
    
        # Node
        #     package='pointcloud_to_laserscan', executable='dummy_pointcloud_publisher',
        #     remappings=[('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud'])],
        #     parameters=[{'cloud_frame_id': 'camera_link', 'cloud_extent': 2.0, 'cloud_size': 500}],
        #     name='cloud_publisher'
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'camera_link']
        # ),

        ########### The parameters are taken from intel realsense data for D435 ###########################################
        ########### Link -> https://github.com/issaiass/realsense2_description/blob/master/urdf/_d435.gazebo.xacro ########
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            # remappings=[
            #             (['scan', '/scan'])],
            parameters=[{
                'target_frame': 'camera_link',
                'transform_tolerance': 0.2,
                'min_height': -0.4, #0.1, #0.0,                                    # min height to sample out the pointcloiud in meters 
                'max_height': +0.8, #1.65, #1.0,                                     # max height to sample out the pointcloiud in meters
                'angle_min': -0.6056, #-1.5708,  # -M_PI/2                  # Taken from intel camera (FOV is 69.4 degrees --> 1.21125 radians)
                'angle_max': 0.6090, #1.5708,  # M_PI/2
                'angle_increment': 0.006101, #0.006,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.3,                                            # min range of intel D435
                'range_max': 10.0,                                           # max range of intel D435
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
