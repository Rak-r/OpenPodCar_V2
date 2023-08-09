from os import environ
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('x', default_value=['0'],
        description='x position'),
    DeclareLaunchArgument('y', default_value=['0'],
        description='y position'),
    DeclareLaunchArgument('z', default_value=['0'],
        description='z position'),
    DeclareLaunchArgument('yaw', default_value=['0'],
        description='yaw position'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('sync', default_value='true',
                          choices=['true', 'false'],
                          description='Run async or sync SLAM'),
    DeclareLaunchArgument('localization', default_value='slam',
                          choices=['off', 'localization', 'slam'],
                          description='Whether to run localization or SLAM'),
    DeclareLaunchArgument('nav2', default_value='true',
                          choices=['true', 'false'],
                          description='Run nav2'),
    # DeclareLaunchArgument('corti', default_value='true',
    #                       choices=['true', 'false'],
    #                       description='Run corti'),
    # DeclareLaunchArgument('cerebri', default_value='true',
    #                       choices=['true', 'false'],
    #                       description='Run cerebri'),
    DeclareLaunchArgument('bridge', default_value='true',
                          choices=['true', 'false'],
                          description='Run bridges'),
    # DeclareLaunchArgument('synapse_ros', default_value='true',
    #                       choices=['true', 'false'],
    #                       description='Run synapse_ros'),
    # DeclareLaunchArgument('synapse_gz', default_value='true',
    #                       choices=['true', 'false'],
    #                       description='Run synapse_gz'),
    # DeclareLaunchArgument('joy', default_value='true',
    #                       choices=['true', 'false'],
    #                       description='Run joy'),
    DeclareLaunchArgument('description', default_value='true',
                          choices=['true', 'false'],
                          description='Run description'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='GZ World'),
    DeclareLaunchArgument(
        'map_yaml',
        default_value=[LaunchConfiguration('world'), '.yaml'],
        description='Map yaml'),
    # DeclareLaunchArgument('cerebri_gdb', default_value='false',
    #                       choices=['true', 'false'],
    #                       description='Run cerebri with gdb debugger.'),
    # DeclareLaunchArgument('uart_shell', default_value='false',
    #                       choices=['true', 'false'],
    #                       description='Run cerebri with UART shell.'),
    DeclareLaunchArgument('spawn_model', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn MR Buggy3 Model'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
#     DeclareLaunchArgument('log_level', default_value='error',
#                           choices=['info', 'warn', 'error'],
#                           description='log level'),
 ]


def generate_launch_description():
    # synapse_ros = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution(
    #         [get_package_share_directory('synapse_ros'), 'launch', 'synapse_ros.launch.py'])]),
    #     condition=IfCondition(LaunchConfiguration('synapse_ros')),
    #     launch_arguments=[('host', ['192.0.2.1']),
    #                       ('port', '4242'),
    #                       ('use_sim_time', LaunchConfiguration('use_sim_time'))]
    # )

    # synapse_gz = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution(
    #         [get_package_share_directory('synapse_gz'), 'launch', 'synapse_gz.launch.py'])]),
    #     condition=IfCondition(LaunchConfiguration('synapse_gz')),
    #     launch_arguments=[('host', ['127.0.0.1']),
    #                       ('port', '4241'),
    #                       ('use_sim_time', LaunchConfiguration('use_sim_time'))]
    # )
    rviz_config_path = PathJoinSubstitution([get_package_share_directory("pod2_navigation"), "rviz2", "nav2_default.rviz"])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
        launch_arguments=[('gz_args', [
            LaunchConfiguration('world'), '.sdf', ' -v 0', ' -r'
            ])]
    )

    # cerebri = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution(
    #         [get_package_share_directory('cerebri_bringup'), 'launch', 'cerebri.launch.py'])]),
    #     condition=IfCondition(LaunchConfiguration('cerebri')),
    #     launch_arguments=[('gdb', LaunchConfiguration('cerebri_gdb')),
    #                       ('vehicle', 'mrbuggy3'),
    #                       ('uart_shell', LaunchConfiguration('uart_shell'))],
    # )

    # joy = Node(
    #     package='joy',
    #     executable='joy_node',
    #     output='screen',
    #     arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    #     condition=IfCondition(LaunchConfiguration('joy')),
    #     parameters=[{
    #         'use_sim_time': LaunchConfiguration('use_sim_time')
    #         }],
    #     remappings=[('/joy', '/cerebri/in/joy')]
    # )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=[
            '/scan' +
             '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        # remappings=[
        #     ('/world/default/model/mrbuggy3/link/lidar_link/sensor/lidar/scan',
        #      '/scan')
        )

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
        arguments=[
            '/model/podcar/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
            ],
        # remappings=[
        #     ('/model/podcar/odometry', '/odom')
        #     ]
        )

    odom_base_tf_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('bridge')),
        arguments=[
           ['/model/podcar/pose' +
            '@tf2_msgs/msg/TFMessage' +
            '[gz.msgs.Pose_V']
        ],
        remappings=[
           (['/model/podcar/pose'], '/tf')
        ])

    pose_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('bridge')),
        arguments=[
           ['/model/podcar/pose' +
            '@tf2_msgs/msg/TFMessage' +
            '[gz.msgs.Pose_V']
        ],
        remappings=[
           (['/model/podcar/pose'],
            '/_internal/sim_ground_truth_pose')
        ])


    # Robot description
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory('pod2_description'), 'launch', 'pod2_description.launch.py'])]),
        condition=IfCondition(LaunchConfiguration('description')),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-world', 'default',
            '-name', 'mrbuggy3',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
            '-file', PathJoinSubstitution([get_package_share_directory(
                'pod2_description'),
                'xacro/tested.urdf'])
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration("spawn_model")))

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory(
        'pod2_navigation'), 'launch', 'navigation.launch.py'])]),
        condition=IfCondition(LaunchConfiguration('nav2')),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    # corti = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution(
    #     [get_package_share_directory('corti'), 'launch', 'corti.launch.py'])]),
    #     condition=IfCondition(LaunchConfiguration('corti')),
    #     launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory(
        'pod2_navigation'), 'launch', 'slam_map_building.launch.py'])]),
        condition=LaunchConfigurationEquals('localization', 'slam'),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('sync', LaunchConfiguration('sync'))])

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory(
        'pod2_navigation'), 'launch', 'localization.launch.py'])]),
        condition=LaunchConfigurationEquals('localization', 'localization'),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('map', PathJoinSubstitution([get_package_share_directory(
                'pod2_navigation'), 'maps', LaunchConfiguration('map_yaml')]))])

    tf_map_odom = Node(
        name='tf_map_odom',
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )


    # Define LaunchDescription variable
    return LaunchDescription(ARGUMENTS + [
        robot_description,
        # synapse_ros,
        # synapse_gz,
        gz_sim,
        # cerebri,
        # joy,
        odom_bridge,
        clock_bridge,
        lidar_bridge,
        odom_base_tf_bridge,
        pose_bridge,
        rviz2,
        spawn_robot,
        nav2,
        # corti,
        slam,
        localization,
        # tf_to_odom
    ])