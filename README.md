# OpenPodCar_V2

OpenPodCar is a donor scooter vehicle which is transformed into autonomous vehicle with the integration of Robot Operating
System (ROS) with the added support of newly released ROS2. The vehilce used for the implementation of our research and experiments can be found at Pihsiang TE-889XLSN hard-canopy scooter (branded in UK as ShopriderTraverso) (found here: https://www.smartscooters.co.uk/Traveso-Mobility-Scooter).
![IMG_6667](https://github.com/Rak-r/OpenPodCar_V2/assets/85680564/d19d6d02-5144-453d-9e30-7fc675c42a9b)



# ROS2 Humble & Gazebo Garden
This project is the complete new port from ROS1 of OpenPodCar1 to ROS2. The full software features the ROS2 Humble version and for simulation is done with new Gazebo.
The OpenPodcar_2 package consists of sub-packages namely; `pod2_description`, `pod2_bringup`, `pod2_navigation`, `pod2_sensor_tools`, `pod2_rtabmap`, `pod2_msgs`.

## System Requirements

1. Ubuntu 22.04


2. ROS2 Humble full desktop install: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html


3. Gazebo Garden (Install): https://gazebosim.org/docs/garden/install_ubuntu


4. The only difference between the Gazebo Fortress and Gazebo Garden is the ros-gz integration package is to build from source for Gazebo Garden while if you are using Gazebo Fortress, the ros_gz package will be installed by binary installation.


5. Gazebo Fortress uses the ignition namepsace when dealing with plugins and setting frame ids for the robot in URDFs or SDFs.


### Testing Installations

1. To test that ROS2 is installed properly.
* Open bashrc and add the folowing and save it. `source /opt/ros/humble/setup.bash`.
* Open two terminals, in first run: `ros2 run demo_nodes_cpp talker`, you should see  `hello` in the console.
* In other terminal, run: `ros2 run demo_nodes_py listener`, you should see `I Heard`.
* In order to keep the nodes communication robust, set the `ROS_DOMAIN_ID` in your bashrc. For example: `export ROS_DOMAIN_ID=0`


2. To test the Gazebo Fortress is installed on the system, in the terminal run: `ign gazebo`.If it launches, you'll see the simulation software window.


3. To test the ros_gz package, source the workspace of the package and try the following command:


* `ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@gz.msgs.StringMsg` and view the topic in other terminal using: `ros2 topic list -t`


## Installation for OpenPodCar_V2

To use this package for testing and running simulations using gazebo and ROS2 follow the below instructions:


1. If using Gazebo Fortress, clone this repo fololowing below commands. 



* Make the new workspace, with src directory. `mkdir -p ros2_gz_ws/src`.
* Clone the repository using: `git clone https://github.com/Rak-r/OpenPodCar_V2.git`



2. After cloning the repository, you should have `pod2_description`, `pod2_bringup`, `pod2_navigation`, `pod2_sensor_tools`, `pod2_yolo`, `pod2_msgs`in your `src` directory.



3. Now, build the packages from the root of the workspace directory using ROS2 package building tool colcon.
* Assuming you are in /src directory: run `cd ..`


* `colcon build --symlink-install`. This will build the packages and the `--symlink-install` is used to make changes in the packages in src directory and also changes in the install dircetory without re-building the package.



4. If everything works well, you will have three directories alomg with `src` named `install`, `build` ad `log`. If colcon build fails to build any packages and shows `stderr` in terminal console, make sure all the dependencies are install correctly.



5. In case of Step 4, try running: `rosdep update && rosdep install --from-paths src --ignore-src -y`.



6. Once the package is build successfully, open bashrc and add : `source <your workspace path>/install/setup.bash`
* **For example** : `source /home/ros2_ws/install/setup.bash` 

# Packages and Descriptions

## Pod2_description

This ROS2 package consists the robot's urdf files in the `xacro` directory, meshes of the robot model, sensors in the `meshes` and the `launch` directory contains the `description.launch.py` and  `pod2_description` file which launches the robot model's URDF and the world file in the Gazebo with a condition to start along the rviz2 node.


The launch file also consists the `ros_gz_bridge` package which is used to establish communication between Gazebo and ROS2. The parameter bridge is created for `/model/podcar/cmd_vel` topic from ROS -> GZ, on this topic the Ackermann system plugin publishes the twist messages.
GZ -> ROS is created for  `LaserScan` and `RGBD` based simulated sensor data which is coming from gazebo sensor system plugin, `/model/podcar/odometry` topic consists of ground-truth odometry data from Gazebo.



### Scripts

This directory in `pod2_description` package consists of intermediate nodes which are used to convert the incoming messages over te topics `/model/podcar/odometry`, `/lidar_scan` to publish on Wall time. This approach is employed to avoid the time realetd issues, tf errors and with an assumption that the simulation and the physical vehicle should work on same time.


This directory in `pod2_description` package consists of intermediate nodes which are used to convert the incoming messages over te topics `/model/podcar/odometry`, `/lidar_scan`, `rgbd_camera/depth` to publish on Wall time. This approach is employed to avoid the time realetd issues, tf errors and with an assumption that the simulation and the physical vehicle should work on same time.

1. `odometry_wall_time.py` handles the ground truth odometry `/model/podcar/odometry` topic from GZ and publishes to ROS2 topic `/odom` with changing the time stamp to wall time.


2. `laser_wall_time.py` handles the `/lidar_scan` topic from GZ laser plugin and publishes to ROS2 topic `/scan` with changing the time stamp to wall time.


3. `RGBD_wall_timer.py` handles the `/rgbd_camera/depth`, `/rgbd_camera/camera_info`, `/rgbd_camera/image`, `rgbd_camera/points` topic from GZ laser plugin and publishes to ROS2 topic `/depth`, `/depth_camera_info`, `/camera/color/image_raw` and `/cloud_in` with changing the time stamp to wall time.


## Pod2_bringup

This ROS2 package utilizes the ROS2 teleop-twist-joy pakcage, in order to control the OpenPodcar using the joystick controller in the simulation as well as in real world physical robot teleoperation. 

Different joystick are tested namely; Logotech Extreme3dPro, generic linux usb joystick, PS2 and XBOX.
To test the joystick is connected to the system run `ls /dev/input`.


In order to use specific joystick you might have to create the `.yaml` config file which can be referenced from (https://github.com/ros2/teleop_twist_joy/tree/humble/config) and the `launch` directory contains the `joy.launch.py` file which launches the `Joy node` and  `teleop_twist_joy_node`.
**For using any custom joystick, you might need to check which buttons and axis does what** 


I recommend using `https://flathub.org/apps/io.gitlab.jstest_gtk.jstest_gtk`. The tool also provide calibrataion for the joystick which mighht be helpful if deploying on the physical vehicle for teleoperation.



## Pod2_navigation

Pod2_navigation package consists of the `launch`, `rviz`, `maps`, `config` directories. 

1. Config directory:
   This includes the `nav2_dwb_smac.yaml` file which includes the parametrers for AMCL, BT_Navigator, Controller server, PLanner server, Global and Local Costmaps, Behaviour servers, Map server.


2. `mapper_params_slam_sync.yaml` and `mapper_params_slam_async.yaml` are the params file which are used to launch the slam-toolbox either in synchronous/asynchronous mode.


3. The launch directory consists of `OpenPodCar_NAV2.launch.py` which uses the default `nav2_bringup` package for launching all the nodes and takes the `parameters from the config directory. It uses AMCL for localization which will also be started.
    
  * `cd <workspace>`


  * `source install/setup.bash`

  * Launch the navigation launch file which starts the nodes; plannar serever, controller server, bt navigator, behavioir server.


  `ros2 launch pod2_navigation OpenPodCar_NAV2.launch.py slam:=false rviz:=true amcl:=true`.


  * To run the slam_toolbox for localization, 


  `ros2 launch pod2_navigation OpenPodCar_NAV2.launch.py slam:=true rviz:=true amcl:=false`.


4. If want to build the map using slam_toolbox, then change the mode to mapping in `mapper_params_online_async.yaml`.


5. The map can be saved either by command-line: ` ros2 run nav2_map_server map_saver_cli -f <name of the map>` or in rviz2 slam_toolbox plugin. More info could be found at: (https://github.com/SteveMacenski/slam_toolbox/tree/humble).


6. In order to save the map with old format (.yaml and .pgm) hit the save map button in rviz2 slam_toolbox plugin and to save in the other format (serilaised), write the name of the map without any extension and  click the serial map button in the rviz2 slamtoolbox_plugin.


7. The package has been tested with the `Sim_1.yaml`, `Sim_2.yaml`, `Sim_3.yaml`with corresponding pgm files.


8. To use the localization with `slam_toolbox`, you have to provide the right path to the map which you are going to use.  When using slam_toolbox for localization, you do not have to provide the map file extension in the `mapper_params_onlie_async.yaml` and just the name.

### Note that slam_toolbox is best suited for LiDAR based robots and struggles with RGBD sensor. The OpenPodCar2 features a single RGBD sensor is tested with slam_toolbox with rigorous parametr tuning both in simulation and real physical vehicle. However, due to less angular FOV, the laser scan matching results in sudden jumps of robot. This has been discussed in SteveMacenski/slam_toolbox#662. To handle this RGBD based slam method RTABMAP is adopted.

## Usage

### Simulation
The new Gazebo Garden is used for the simulation of OpenPodCar_v2. The new gazebo features more functionalities with enhanced inetrface. As our robot behaves as car-like robot and features Ackermann-Steering kinematics. To maintain this behaviour in simulation the new gazebo now has an Ackermann system plugin which could be used according the robot configuartions. The plugin outputs standard `Twist` messages of field `linear.x` and `angular.z`. This also outputs the odometry information which might not be the correct odometry for the whole robot instead it is the odometry information for steering.

The current repository features the ROS2 Humble with Gazebo garden. To use the ROS2 Humble packages with Gazebo Fortress, switch to the Fortress branch https://github.com/Rak-r/OpenPodCar_V2/tree/Fortress.

[Podcar_V2_GZ_garden.webm](https://github.com/Rak-r/OpenPodCar_/assets/85680564/26ea85f9-a46d-4f53-b81a-1f23425ab1f7)

#### If you want to launch the PodCar with Lidar enabled, run the below launch file:

  * Launch without Rviz : ros2 launch pod2_decsription pod2_description_Lidar.launch.py scan_node:=true rgbd_node:=false

  * Launch along with Rviz: ros2 launch pod2_description pod2_description.launch.py rviz:=true scan_node:=true rgbd_node:=false

#### If you want to launch the PodCar with depth camera enabled, run the below launch file:

  *  Launch without Rviz : ros2 launch pod2_decsription pod2_description_Depth.launch.py scan_node:=false rgbd_node:=true

  *  Launch along with Rviz: ros2 launch pod2_description pod2_description_Depth.launch.py rviz:=true scan_node:=false rgbd_node:=true


This above will launch the simulation in gazebo and don't forget to turn on the play pause button to run the simulation. To view the active topics in ros2, use ros2 topic list -t in the terminal window. To view active topics in gazebo, use gz topic -l in the terminal window. Podcar_V2_GZ_garden.webm

### Simulation Teleoperation and Autonomous operation

  *  Start gamepad to publish twist to gazebo: ros2 launch pod2_bringup generic_gamepad.launch.py

  *  Start the rtabmap rgbd odometry and slam:  ros2 launch pod2_rtabmap rtabmap.launch.py

  *  Launch NAV2 stack: ros2 launch pod2_navigation OpenPodCar_NAV2.launch.py slam:=false amcl:=false

After mapping, if want to start the NAV2 stack in pre-build map, rtabmap can be started in localization mode. In order to autonomous drive while mapping the above could be just followed.

### Physical vehicle Tele-operation & Autonomous operation

To launch the physical OpenPodCar2 with teleoperation mode, the higher-level incoming game-pad commands as Twist message linear.x, angualr.z are converted to R4 protocol message which controls the main driver motor for forward and backward movement and linear actuator for controlling the steering for the OpenPodCar2.

To start the physical vehicle for tele-operation, after building the OpenPodCar2 packaghe from following above instruction.

  1. Start the R4-ROS2 communication nodes using the launch file:

ros2 launch pod2_bringup R4_ros.launch.py teleop_node:=true

  2. Launch the robot model: ros2 launch pod2_description pod2_description.launch.py

  3.  Start the camera sensor along with point to laserscan node: ros2 launch pod2_sensor_tools point_to_scan.launch.py

  4.  Start the rtabmap rgbd odometry and slam:  ros2 launch pod2_rtabmap rtabmap.launch.py

  5.  Launch NAV2 stack: ros2 launch pod2_navigation OpenPodCar_NAV2.launch.py slam:=false amcl:=false

After mapping, if want to start the NAV2 stack in pre-build map, rtabmap can be started in localization mode. In order to autonomous drive while mapping the above could be just followed.

## Node graphs


### Full NAV2 stack with low-level hardware nodes and ROS2 on Physical vehicle

![Podcar_Nav2_node_graph](https://github.com/Rak-r/OpenPodCar_V2/assets/85680564/423534b6-b491-46bf-8d37-8af7f39bb623)



## Images & Videos

### OpenPodcar2 for outdoor operation

![Podcar2_outdoor](https://github.com/Rak-r/OpenPodCar_V2/assets/85680564/36b9ad93-4df8-4b74-8477-a95e82b17212)


### Tight Indoor drive Navigation2 Stack

 

https://github.com/Rak-r/OpenPodCar_V2/assets/85680564/c05b5af3-1316-4407-bb80-9b6b9504561f


