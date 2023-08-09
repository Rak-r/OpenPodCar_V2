import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
import launch

def generate_launch_description():
     
	ros_gz_pkg = get_package_share_directory('ros_gz_sim')
	
	#use_sim_time = LaunchConfiguration('use_sim_time')
	path_to_urdf = PathJoinSubstitution([FindPackageShare("pod2_description"), "xacro", "tested.urdf"])
	# world_path = os.path.join(FindPackageShare("pod2_description"), 'xacro', 'my_model.sdf')
	gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(ros_gz_pkg, 'launch', 'gz_sim.launch.py')), launch_arguments={'gz_args': '-r empty.sdf'}.items(),)
    
	rviz_config_path = PathJoinSubstitution([FindPackageShare("pod2_description"), "rviz2", "description.rviz"])
	
	robot_description = ParameterValue(
        Command(['xacro ', str(get_package_share_path('pod2_description') / 'xacro/tested.urdf')]),
        value_type=str)
	# gazebo = [
    #     ExecuteProcess(
    #         condition=launch.conditions.IfCondition(run_headless),
    #         cmd=['ruby', FindExecutable(name="ign"), 'gazebo',  '-r', '-v', gz_verbosity, '-s', '--headless-rendering', world_path],
    #         output='screen',
    #         additional_env=gz_env, # type: ignore
    #         shell=False,
    #     ),
    #     ExecuteProcess(
    #         condition=launch.conditions.UnlessCondition(run_headless),
    #         cmd=['ruby', FindExecutable(name="ign"), 'gazebo',  '-r', '-v', gz_verbosity, world_path],
    #         output='screen',
    #         additional_env=gz_env, # type: ignore
    #         shell=False,
    #     )]

	spawn = Node(
		package='ros_gz_sim',
		executable='create',
		arguments=['-name', 'podcar',
	     '-topic', '/robot_description',
	    '-x', '0', '-y', '0', '-z', '0.5'],
	     output='screen',)
	
    
	return LaunchDescription([
	DeclareLaunchArgument(
            name='urdf', 
            default_value=path_to_urdf,
            description='URDF path'
        ),
        gazebo,
	    spawn,
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
            # package='joint_state_publisher',
            # executable='joint_state_publisher',
            # name='joint_state_publisher',
            # condition=IfCondition(LaunchConfiguration("publish_joints"))
            # parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}
            # ]),
       
        # creating the (ROS-->GZ) bridge for podcar cmd-vel topic, (GZ-->ROS) for odometry topic, lidar data
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
            '/model/podcar/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
            ],
            remappings=[
		    (['/model/podcar/tf'], '/tf')
            ]
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

