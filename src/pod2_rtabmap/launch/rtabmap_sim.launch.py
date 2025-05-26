
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    # parameters={
    #       'frame_id':'base_link',
    #       'use_sim_time':use_sim_time,
    #       'subscribe_depth':True,
    #       'approx_sync': True,
    #     #   'approx_sync_max_interval': 0.01,
    #       'publish_tf': False,
    #       'Reg/Force3DoF':'true',
    #       'publish_null_when_lost': False,
    #       'Grid/RangeMin': '0.3',
    #       'Grid/RangeMax': '10.0',
    #       'Grid/RayTracing': 'true',
    #       'Grid/FromDepth': 'true',
    #       'queue_size': 10,
    #     #   'max_update_rate': 0.05,
    #     #   'min_update_rate':0.1,
          
    #     #   'use_action_for_goal':True,
    #     #   'qos_image':qos,
    #     #   'qos_imu':qos,
    #     #   'Reg/Force3DoF':'true',
    #     #   'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    # }

    param = {
          'frame_id': 'base_link',
          'use_sim_time':use_sim_time,
          'subscribe_depth':False,
          'subscribe_scan':True,
        #   'subscribe_scan_cloud': True,
          'approx_sync': True,
        #   'approx_sync_max_interval': 0.01,
          'publish_tf': True,
          'Reg/Force3DoF':'true',                                                       # publish only 2D pose
          'map_always_update': True,                                                    # always update the map not only when the robot moves
          'Grid/RangeMin': '0.3',                                                       # min range of depth
          'Grid/RangeMax': '5.0',                                                       # max range of depth (intel D435 looses accuaracy at 10 m)
          'Grid/RayTracing': 'true',                                                    # enable ray-tracing to clear out cells and mark as free space in occupancy grid map
          'Grid/FromDepth': 'true',                                                    # generate occupancy map using depth image 
          'RGBD/OptimizeMxError': '0',                                                  # to reject the wrong loop closures
          'RGBD/ProximityBySpace': 'false',
          'RGBD/AngularUpdate': '0.01',
          'RGBD/LinearUpdate': '0.01',
        #   'use_action_for_goal':True,
        #   'qos_image':qos,
        #   'qos_imu':qos,
          'Optimizer/GravitySigma':'0',                                                  # Disable imu constraints (we are already in 2D)
          'Rtabmap/DetectionRate': '10.0',
    }

    remappings=[
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/depth_camera_info'),
          ('depth/image', '/depth'),
          # ('/odom', '/rtabmap_odom'),
          
                                                         # odmetry topic to subscribe to
        #   ('scan_cloud', '/cloud_in')                                                 # if want to use pointcloud2 message from the camera
          ]                                                

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='False',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
            
        DeclareLaunchArgument(
            'localization', default_value='False',
            description='Launch in localization mode.'),

        # Nodes to launch
       
        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[param],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
        # ),

        #  Node(
        #     package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        #     parameters=[param],
        #     remappings=remappings),
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        #  Node(
        #      package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #      parameters=[param],
        #      remappings=remappings),
    ])