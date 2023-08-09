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
    
	world_path = os.path.join(get_package_share_directory('pod2_description'))
	path_to_urdf = PathJoinSubstitution([FindPackageShare("pod2_description"), "xacro", "tested.urdf"])
	
	rviz_config_path = PathJoinSubstitution([FindPackageShare("pod2_description"), "rviz2", "description.rviz"])
	
	robot_description = ParameterValue(
        Command(['xacro ', str(get_package_share_path('pod2_description') / 'xacro/tested.urdf')]),
        value_type=str)

	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
	gazebo_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
		os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
	launch_arguments={'gz_args': PathJoinSubstitution([
		world_path, 'xacro', 'model.sdf'
    ]), 'use_sim_time':'true'  }.items(),
    )
	spawn = Node(
		package='ros_gz_sim',
		executable='create',
		arguments=['-name', 'podcar',
	     '-topic', '/robot_description',
	    '-x', '0', '-y', '0', '-z', '0.5'],
	     output='screen',
	     
    )
	# creating the (ROS-->GZ) bridge for podcar cmd-vel topic, (GZ-->ROS) for odometry topic, lidar data
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
            default_value='true',
            description='Use simulation time'
        ),
         Node(
		package ='joint_state_publisher',
		executable='joint_state_publisher',
		name='joint_state_publisher',
		parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
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
        ),
        Node(
		package='ros_gz_bridge',
		name = 'ros_gz_bridge',
		executable='parameter_bridge',
		output='screen',
		parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
		arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
	     '/model/podcar/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
	     '/model/podcar/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
	 '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
     'model/podcar/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
     remappings=[
		    (['/model/podcar/tf'], '/tf'),
		    (['/model/podcar/odometry'], '/odom')
            ]
    ),
    gazebo_sim,
	    spawn,
    

    ])
