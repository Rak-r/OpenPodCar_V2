
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_link',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          'approx_sync': True,
        #   'approx_sync_max_interval': 0.01,
          'publish_tf': False,
          'Reg/Force3DoF':'true',
          'publish_null_when_lost': False,
          'Grid/RangeMin': '0.3',
          'Grid/RangeMax': '10.0',
          'Grid/RayTracing': 'true',
          'Grid/FromDepth': 'true',
          'queue_size': 10,
        #   'max_update_rate': 0.05,
        #   'min_update_rate':0.1,
          
        #   'use_action_for_goal':True,
        #   'qos_image':qos,
        #   'qos_imu':qos,
        #   'Reg/Force3DoF':'true',
        #   'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    slam_param = {
        #   'odom_frame_id':'odom',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          
          'approx_sync': False,
        #   'approx_sync_max_interval': 0.01,
          'publish_tf': True,
          'Reg/Force3DoF':'true',
          
          'Grid/RangeMin': '0.3',
          'Grid/RangeMax': '10.0',
          'Grid/RayTracing': 'true',
          'Grid/FromDepth': 'true',
          
        #   'use_action_for_goal':True,
        #   'qos_image':qos,
        #   'qos_imu':qos,
        #   'Reg/Force3DoF':'true',
        #   'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    remappings=[
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/depth'),
          ('/odom', '/vo')]                                                # to subscribe to ekf odometry

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
       
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[parameters],
            remappings=remappings),

        # SLAM mode:
        # Node(
        #     condition=UnlessCondition(localization),
        #     package='rtabmap_slam', executable='rtabmap', output='screen',
        #     parameters=[slam_param],
        #     remappings=[('/odom', '/odometry/filtered'),
				  
		# 		 ],
        #     arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # # Localization mode:
        # Node(
        #     condition=IfCondition(localization),
        #     package='rtabmap_slam', executable='rtabmap', output='screen',
        #     parameters=[parameters,
        #       {'Mem/IncrementalMemory':'False',
        #        'Mem/InitWMWithAllNodes':'True'}],
        #     remappings=remappings),

        #  Node(
        #      package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #      parameters=[parameters],
        #      remappings=remappings),
    ])