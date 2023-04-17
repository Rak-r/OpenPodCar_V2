import rclpy
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
def generate_launch_description():
    
     slam_config_path = PathJoinSubstitution(FindPackageShare("pod2_navigation"), "config", "mapper_params_online_sync.yaml")
   
     slam_rviz_path = PathJoinSubstitution(FindPackageShare("pod2_navigation"), "rviz2", "slamtoolbox.rviz")
     return LaunchDescription([
        DeclareLaunchArgument(name='sim', default_value='false', description='enable use_sim_time to true'),
        DeclareLaunchArgument(name='rviz', default_value='false', description='run rviz2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_config_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                slam_param_name: slam_config_path
            }.items()),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', slam_rviz_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        )
    ])
