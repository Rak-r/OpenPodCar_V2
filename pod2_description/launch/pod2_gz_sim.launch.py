import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():


    rviz_config_path = PathJoinSubstitution([FindPackageShare("pod2_description"), "rviz2", "description.rviz"])
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ackermann_steering.sdf'}.items(),
    )

    '''run the rivz along with gazebo garden'''
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    '''set the bridge using the node from launch_ros module'''

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/vehicle_blue/cmd_vel@geomtry_msgs/msg/Twist[ignition.msgs.Twist',
                   '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry]ignition.msgs.Odometry'],
        parameters=[{'qos_overrides./model/vehicle_blue.subscriber.reliability': 'reliable',
                     'qos_overrides./model/vehicle_green.subscriber.reliability': 'reliable'}],
        output='screen'
    )

    return LaunchConfiguration([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true', description='launch rviz'),
        bridge,
        rviz
    ])
