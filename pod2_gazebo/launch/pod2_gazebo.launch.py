import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    description_path = os.path.join(get_package_share_directory("pod2_description"), "launch", "description.launch.py")

    world_path = os.path.join(get_package_share_directory("gazebo_ros"), "worlds", "empty.world")

    joy_path = os.path.join(get_package_share_directory("pod2_bringup"), "launch", "joy.launch.py")

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    

    return LaunchDescription([
        DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Gazebo world'
        ),

        ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so', world_path],
        output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", 'robot_description', "-entity", "podcar"]
        ),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_path), 
        launch_arguments={'use_sim_time':use_sim_time, 'publish_joints':'false',}.items()),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_path),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
            )
        )
        
    ])
	
