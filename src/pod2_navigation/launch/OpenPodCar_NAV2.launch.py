import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('pod2_navigation')
    launch_dir = os.path.join(bringup_dir, 'launch')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    rviz_config_path = PathJoinSubstitution([FindPackageShare("pod2_navigation"), "rviz2", "OpenPodCar.rviz"])
    

    slam = LaunchConfiguration('slam')                                                       # create launch configuration variable for slam toolbox
    amcl = LaunchConfiguration('amcl')                                                       # create launch configuration variable for amcl for localization on pre-built map

    # map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')
    # map_file_path = os.path.join(
    #     get_package_share_directory('pod2_navigation'),
    #     'maps',
    #     'empty_map.yaml'
    # )

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                    #    'map_server',
                       'waypoint_follower']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/cmd_vel', '/cmd_vel_nav2')
                ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart
        }

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    # whether to launch slam toolbox along with NAV2 
    slam_toolbox_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'async_slam.launch.py')),condition = IfCondition(slam))


    #whether to start the AMCL based localization in case of pre-built map of the enviroment
    amcl_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization.launch.py')), condition= IfCondition(amcl))

    return LaunchDescription([

        slam_toolbox_launch,
        amcl_launch,
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),

        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
        # nav2_dwb_smac nav2_game_theory
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'config', 'nav2_dwb_smac.yaml'), description='Full path to the ROS2 parameters file to use'),

    
        DeclareLaunchArgument('map_subscribe_transient_local', default_value='true', description='Whether to set the map subscriber QoS to transient local'),

        DeclareLaunchArgument('slam', default_value='False', description='Whether run a SLAM'),

        DeclareLaunchArgument('amcl', default_value='False', description = 'Whether to launch AMCL localizer'),

        DeclareLaunchArgument(name='rviz', default_value='false',description='Run rviz'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            #prefix=['xterm -e gdb -ex run --args'],
            output='screen',
            parameters=[configured_params],
            remappings=remappings
            ),

        # Node(
        # package='nav2_map_server',
        # executable='map_server',
        # output='screen',
        # parameters=[{'yaml_filename': map_file_path}]),
        
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            #prefix=['xterm -e gdb -ex run --args'],
            output='screen',
            parameters=[configured_params],
            remappings=remappings
            ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            #prefix=['xterm -e gdb -ex run --args'],
            output='screen',
            parameters=[configured_params],
            remappings=remappings
            ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            #prefix=['xterm -e gdb -ex run --args'],
            output='screen',
            parameters=[configured_params],
            remappings=remappings
            ),
        
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            #prefix=['xterm -e gdb -ex run --args'],
            output='screen',
            parameters=[configured_params],
            remappings=remappings
            ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            #prefix=['xterm -e gdb -ex run --args'],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes},
                        # {'bond_timeout': 0.0}
                        ]),
       
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        
    ])