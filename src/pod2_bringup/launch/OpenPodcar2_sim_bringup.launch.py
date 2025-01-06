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

    ######## Define paths to packages ########
    ##########################################

    path_to_teleop_joy_launch = get_package_share_directory('pod2_bringup')

    path_to_description = get_package_share_directory('pod2_description')

    path_to_sensor_launch = get_package_share_directory('pod2_sensor_tools')

    path_to_rtabmap = get_package_share_directory('pod2_rtabmap')

    path_to_navigation = get_package_share_directory('pod2_navigation')
    
    ####### Include the required launch files ########
    ##################################################

    # launch the URDF of the vehicle(OpenPodCar_2)
    pod2_description = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(path_to_description, 'launch', 'pod2_description.launch.py')),)

    # launch the teleop twist joy after connecting the gamepad witht the system via usb
    teleop_joy = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(path_to_teleop_joy_launch, 'launch', 'joy.launch.py')),
                                            launch_arguments={'teleop_node':LaunchConfiguration('teleop_node')}.items())

    # launch the sensor nodes (intel d435 along with pointcloud to scan conversion for NAV2 costmaps requirement).
    pod2_sensor = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(path_to_sensor_launch, 'launch', 'point_to_scan.launch.py')),)

    # launch the RTABMAP stack for both rgbd odometry and SLAM
    pod2_rtabmap = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(path_to_rtabmap, 'launch', 'rtabmap_sim.launch.py')),)

    pod2_navigation = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(path_to_navigation, 'launch', 'nav2_sim.launch.py')),)
    


    ld.add_action(pod2_description)
    ld.add_action(teleop_joy)
    ld.add_action(pod2_sensor)
    ld.add_action(pod2_rtabmap)
    ld.add_action(pod2_navigation)
    
    return ld