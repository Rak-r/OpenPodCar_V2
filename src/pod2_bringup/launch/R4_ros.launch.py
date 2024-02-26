'''Add chmod +x to run the executable using ros2 run command'''

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
    ld = LaunchDescription()

    path_to_teleop_node = get_package_share_directory('pod2_bringup')
    exclude_teleop_node = DeclareLaunchArgument('teleop_node', default_value='false', description='Do not start the teleop node')
# establishies the communication between R4 and ROS2 via UDP sockets
    
    teleoperation = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(path_to_teleop_node, 'launch', 'joy.launch.py')),
		launch_arguments={'teleop_node':LaunchConfiguration('teleop_node')}.items())
    
    Websockets_node = Node(
        package='pod2_bringup',
        name='Websockets_node',
        executable= 'R4-Websockets-Client.py',
        output = 'screen'
    )

    R4_Receiver = Node(
        package='pod2_bringup',
        name='R4_Receiver',
        executable='R4_Receiver.py',
        output= 'screen'
    )

    R4_Publisher = Node(
        package='pod2_bringup',
        name='R4_Publisher',
        executable='R4_Publisher.py',
        output= 'screen'
    )

# R4_ROS2 control nodes which subscribes to cmd_vel and publishes to R4_Protocol messages to control the steering and main hub motor for fwd/bkwd drive

    Podcar_Steer_node = Node(
        package='pod2_bringup',
        name='Podcar_Steer_control',
        executable='Podcar_Steer_node.py',
        output = 'screen',
        
    )
    Podcar_motor_driver = Node(
        package='pod2_bringup',
        name='motor_driver',
        executable='Podcar_motor_driver.py',
        output = 'screen',
        
    )


    ld.add_action(teleoperation)
    ld.add_action(Websockets_node )
    ld.add_action(Podcar_Steer_node)
    ld.add_action(Podcar_motor_driver)
    ld.add_action(R4_Receiver)
    ld.add_action(R4_Publisher)

    return ld