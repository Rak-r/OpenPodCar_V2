# OpenPodCar_2

OpenPodCar is a donor scooter vehicle which is transformed into autonomous vehicle with the integration of Robot Operating
System (ROS) with the added support of newly released ROS2. The vehilce used for the implementation of our research and experiments can be found at Pihsiang TE-889XLSN hard-canopy scooter (branded in UK as ShopriderTraverso) (found here: https://www.smartscooters.co.uk/Traveso-Mobility-Scooter).

# ROS2 Humble & Gazebo Garden
This project is the complete new port from ROS1of OpenPodCar1 to ROS2. The full software features the ROS2 Humble version and for simulation is done with new Gazebo Garden.
The OpenPodcar_2 package consists of sub-packages namely; `pod2_description`, `pod2_bringup`, `pod2_navigation`.

## Pod2_description

This ROS2 package cosnists the robot's urdf files in the `xacro` directory, meshes of the robot model, sensors in the `meshes` and the `launch` directory contains the `description.launch.py` file which launches the robot model's URDF and the world file in the Gazebo garden with a condition to start along the rviz2 node.
The launch file also consists the `ros_gz_bridge` package which is used to establish communication between Gazebo and ROS2. The parameter bridge is created for `/model/podcar/cmd_vel` topic from ROS -> GZ, on this topic the Ackermann system plugin publishes the twist messages.
GZ -> ROS is created for `/clock`, `/lidar_scan` topic which is coming from gazebo sensor system plugin, `/model/podcar/odometry` topic consists of ground-truth odometry data from Gazebo.
### Usage

* Launch without Rviz : `ros2 launch pod2_decsription description.launch.py`

* Launch along with Rviz: `ros2 launch pod2_description description.launch.py rviz:=true`
This launch will launch the simulation in gazebo and don't forget to turn on the play pause button to run the simulation. 
To view the active topics in ros2, use `ros2 topic list -t` in the terminal window.
To view active topics in gazebo, use `gz topic -l` in the terminal window.

### Scripts

This directory in `pod2_description` package consists of intermediate nodes which are used to convert the incoming messages over te topics `/model/podcar/odometry`, `/lidar_scan` to publish on Wall time. This approach is employed to avid the time realetd issues, tf errors and with an assumption that the simulation and the physical vehicle should work on same time.
1. `odometry_wall_timer.py` handles the ground truth odometry `/model/podcar/odometry` topic from GZ and publishes to ROS2 topic `/odom` with changing the time stamp to wall time.
2. `laser_wall_timer.py` handles the `/lidar_scan` topic from GZ laser plugin and publishes to ROS2 topic `/scan` with changing the time stamp to wall time.
3. `transform_broadcaster.py` subcribes to the `/odom` topic published by the odometry_wall_timer node. The transform message field is made to publish wall time.

## Pod2_bringup

This ROS2 package utilizes the ROS2 teleop-twist-joy pakcage, in orderto control the OpenPodcar using the joystick controller in the simulation. 
Different joystick are tested namely; Logotech Extreme3dPro, standard linux usb joystick and PS2.
To test the joystick is connected to the system run `ls /dev/input`.
In order to use specific joystick you might hav to create the `.yaml` config file which can be referenced from (https://github.com/ros2/teleop_twist_joy/tree/humble/config) and the `launch` directory contains the `joy.launch.py` file which launches the `Joy node` and  `teleop_twist_joy_node`.
For using any custom joystick, you might need to check which buttons and axis does what. I recommend using (https://flathub.org/apps/io.gitlab.jstest_gtk.jstest_gtk). The tool also provide calibertaion for the joystcik which mighht be helpful if deploying on the physical vehicle for teleoperation.

## Pod2_navigation

Pod2_navigation package consists of the `launch`, `rviz`, `maps`, `config` directories. 

1. Config directory:
   This includes the `nav2_params.yaml` file which includes the parametrers for AMCL, BT_Navigator, Controller server, PLanner server, Global and Local Costmaps, Behaviour servers, Map server.
   `mapper_params_slam_sync.yaml` and `mapper_params_slam_async.yaml` are the params file which are used to launch the slam-toolbox either in synchronous/asynchronous mode.
3. Launch directory:
   * The launch directory consists of nav2_launch.py which uses the default `nav2_bringup` package for launching all the nodes and takes the `nav2_params` from the config directory. It uses AMCL for localization which will also be started.
     Command:
     `cd <workspace>
     source install/setup.bash
     ros2 launch pod2_navigation nav2_launch.py'
   * To run the slam_toolbox for localization, we have to turn off the AMCL and map server. For this, another launch file `navigation.launch.py` is there which launches the required nodes. Now run the slam_toolbox using the launch file.
     `ros2 launch pod2_navigation slam_toolbox_sync.launch.py`.
4. If want to build the map using slam_toolbox, then change the mode to mapping in `mapper_params_online_sync.yaml`.
5. The map can be saved either by command-line: ` ros2 run nav2_map_server map_saver_cli -f <name of the map>` or in rviz2 slam_toolbox plugin. More info could be found at: (https://github.com/SteveMacenski/slam_toolbox/tree/humble)


   
