'''The launch file for starting the gazebo simulation including the robot urdf model and the world.
There is condition set for launching rviz to visualize the robot in ROS side as well to verify the model is loaded correctly.
Make sure to turn the simulation play pause button to run the simulation.'''

'''Usage:
cd ros_ws
source install/setup.bash
ros2 launch pod2_description pod2_description.launch.py rviz:=true scan_node:= false rgbd_node:=true
'''

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
    
    #provide the path for the world file, urdf, rviz
	world_path = os.path.join(get_package_share_directory('pod2_description'))
	path_to_urdf = PathJoinSubstitution([FindPackageShare("pod2_description"), "xacro", "OpenPodCar_V2_Depth.urdf"])
	path_to_realtime_nodes = get_package_share_directory('pod2_description')
	rviz_config_path = PathJoinSubstitution([FindPackageShare("pod2_description"), "rviz2", "description.rviz"])
	
	robot_description = ParameterValue(
        Command(['xacro ', str(get_package_share_path('pod2_description') / 'xacro/OpenPodCar_V2_Depth.urdf')]),
        value_type=str)
    
    #get the package path for the above files
	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
	
	gazebo_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
		os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
	launch_arguments={'gz_args': PathJoinSubstitution([
		world_path, 'xacro', 'empty.sdf'
    ]), 'use_sim_time':'false'}.items(),)
	
    # Stop the scan publisher node while launching to avoid topic hindrence between depthimage_to_laserscan and scan_publisher_node
	exclude_scan_node = DeclareLaunchArgument('scan_node', default_value='false', description='Do not start the scan publisher node')
	sim_2_real = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(path_to_realtime_nodes, 'launch', 'podcar_sim2real.launch.py')),
		launch_arguments={'scan_node':LaunchConfiguration('scan_node')}.items())
    
    
    #define the required nodes for simulation and rviz2
	spawn = Node(
		package='ros_gz_sim',
		executable='create',
		arguments=['-name', 'podcar',
	     '-topic', '/robot_description',
	    '-x', '0', '-y', '0', '-z', '0.5'],
	     output='screen',
	    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
	'''For using ROS2 with Gazebo garden, needed to create the ros_gz_bridge to transfer topics between the two systems
	Making the bridge from ROS ->GZ for the twist message coming from Ackermann plugin of gazebo and
    GZ -> ROS  for the messages namely; clock, groundtruth odometry, Depth and rgb data coming from simulated sensor plugin from GZ'''
	ros_gz_bridge = Node(
		package='ros_gz_bridge',
		name = 'ros_gz_bridge',
		executable='parameter_bridge',
		output='screen',
		arguments=[
	     '/model/podcar/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
	    '/model/podcar/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
		 '/model/podcar/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
	 '/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
	 '/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
	 'rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
	 
     ],
     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
     remappings=[('/model/podcar/cmd_vel', 'cmd_vel'),
				  
				 ] 
    )
	return LaunchDescription([
	exclude_scan_node,
	
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
            name='scan_pub', 
            default_value='false',
            description='Runs the scan_publisher_node'
        ),
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
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
                    'robot_description': robot_description}]
		    ),         
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        gazebo_sim,
	    spawn,
	    ros_gz_bridge,
		sim_2_real,
	    
    ])
