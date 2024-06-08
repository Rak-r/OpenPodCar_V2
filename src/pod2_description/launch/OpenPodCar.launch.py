import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
	
	path_to_urdf = PathJoinSubstitution([FindPackageShare("pod2_description"), "xacro", "OpenPodCar_V2_Depth.urdf"])
	
	rviz_config_path = PathJoinSubstitution([FindPackageShare("pod2_navigation"), "rviz2", "OpenPodCar.rviz"])
	
	robot_description = ParameterValue(
        Command(['xacro ', str(get_package_share_path('pod2_description') / 'xacro/Openpodcar.urdf')]),
        value_type=str)

	
	return LaunchDescription([
	DeclareLaunchArgument(
            name='urdf', 
            default_value=path_to_urdf,
            description='URDF path'
        ),
        
        DeclareLaunchArgument(
            name='publish_joints', 
            default_value='true',
            description='Launch joint_states_publisher'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Use simulation time'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration("publish_joints"))
            # parameters=[
            #     {'use_sim_time': LaunchConfiguration('use_sim_time')}
            # ] #since galactic use_sim_time gets passed somewhere and rejects this when defined from launch file
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': robot_description}]),
                
            
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])