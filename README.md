# OpenPodCar_V2

OpenPodCar is a donor scooter vehicle which is transformed into autonomous vehicle with the integration of Robot Operating
System (ROS) with the added support of newly released ROS2. The vehicle used for the implementation of our research and experiments named as Pihsiang TE-889XLSN hard-canopy scooter (branded in UK as ShopriderTraverso) (found here: https://www.smartscooters.co.uk/Traveso-Mobility-Scooter).
![IMG_6667](https://github.com/Rak-r/OpenPodCar_V2/assets/85680564/d19d6d02-5144-453d-9e30-7fc675c42a9b)

This is an Open Source Hardware and Software platform for Autonomous driivng research applications.



#### Full credit for development and testing of hardware platform named R4 board (R4: rapid reproducible robotics research open hardware control system https://arxiv.org/abs/2402.09833) is given to Mr. Chris Waltham (University of Lincoln,UK) and Dr. Charles Fox (https://staff.lincoln.ac.uk/4311dbb7-1b10-4844-bba9-20f527168e7b)  (University of Lincoln,UK).


## Table of Contents
I. [General Info](#general-info)

II. [Software descritpion](#software-description)

III. [Bill of Materials](#bom)

IV. [Software setup](#software-requirement)

V. [Testing installation](#testing-installation)

VI. [Installation for Openpodcar_v2](#installation-for-openpodcar_v2)

VII. [Docker support for OpenPodcar2](#docker-support-for-OpenPodcar2)

VIII. [Calibration](#calibration)

IX. [Operator instructions](#operator-instructions)


## I. <a name="general-info"></a> General Info

### ROS2 Humble & Gazebo Fortress
This project is the complete new port from ROS1 of OpenPodCar1 to ROS2. The full software features the ROS2 Humble version and for simulation is done with new Gazebo.
The OpenPodcar_2 package consists of sub-packages namely; `pod2_description`, `pod2_bringup`, `pod2_navigation`, `pod2_sensor_tools`, `pod2_yolo`, `pod2_msgs`.

## II. <a name="software-description"></a> Software description

OpenPodcar2 uses a new software stack based on Robot Operating System version 2 (ROS2).  ROS2 is firstly a middleware system implementing publish-subscribe message passing between nodes across a TCP/IP network. Messages are published on named topics which can be subscribed to by any nodes interested in them. ROS2 is secondly a software ecosystem of state-of-the-art implementations of drivers for robots and simulators and of standard robotics algorithms. 

### Kinematic Control

OpenPodcar2 is as an Ackermann-steered vehicle.   The ROS2 community uses `Twist` as the standard for sending mobile robot motion commands (ROS REP-119), so OpenPodcar2 takes `cmd_vel:Twist` commands as inputs.    ROS2 also provides an Ackermann drive message `ackermann_msgs/msg/AckermannDrive` which contains fields for speed and steering angle.

### Manaul Teleoperation

* An Xbox controller is used as the manual controller.  `joy` is a standard ROS2 package which consists of node to interface this controller (or many others) to send messages of type `joy` on the topic `/joy`.

* `telelop_twist_joy` is a standard ROS2 node which converts `joy` messages to `Twist` type `cmd_vel` messages.  It includes an additional DMH configured on a button on the controller used via standard ROS2 YAML parameter file (RL for XBox). This DMH button must be held down on the controller in order for any other controls to have an effect.

### Sensors

OpenPodcar2 uses a single depthcam mounted on the front of the vehicle. This  outputs RGB and depth image data as standard ROS2 `Image` messages, along with pointcloud data over `PointCloud2` messages. `PointCloud2` messages are produced directly by the RGBD camera, as for lidar sensors.

### Localization and mapping

* The ROS2 ecosystem provides several alternative tools for SLAM.   We use RTAB-Map, a 3D voxel based SLAM which is specialised for use with RGBD cameras. The RTAB-Map package also provides the visual odometry named as RGBD odometry which is deployed in the vehicle to provide the pose information and `odom` to `base_link` transform. This has shown reliable results for indoor operations.


* RTAB-Map SLAM package is used to perform the the mapping and localization which takes the `odom` to `base_link` transform as input and corrects the pose information of OpenPodcar2 in the map frame. 


* The Intel Realsense D435 mounted on the vehicle has range of 10m but offers good accuracy for 5m range. The system checked with both ranges and default is set to 5m.

### Navigation

The ROS2 ecosystem includes a navigation stack,  Navigation2 (nav2), designed to facilitate the smooth navigation of a vehicle from one point to another (from point A to B) while avoiding obstacles along the path.  OpenPodcar2 utilizes A* hybrid as global path planner, DWB controller server, specific behaviour servers (wait, back).


### Pedestrian detection and tracking

* The RGBD camera is used for pedestrian detection and tracking as well as for SLAM.   An off-the-shelf ROS2-wrapped YOLOv8 (https://github.com/Rak-r/yolov8_ros2_OpenPodCarV2.git) is linked to the camera to perform and report pedestrian and vehicle detection and tracking in 3D space. 


* YOLOv8 2D detection reports bounding box co-ordinates as ($x$ center, $y$ center, width of the box, height of the box); class name, class ID, confidence score are also extracted. YOLOv8 includes 2D tracking on these detection with BotSORT and ByteTrack which provides acceptable real-time performance but suffers from false re-id errors.  ByteTrack is used as default here. 

* An Intel depth camera ROS2 wrapper is provided by the Intel Realsense SDK (https://www.intelrealsense.com/sdk-2/) which publishes the data over `/camera_image_raw` for RGB image, `/depth` for the depth image, and `/depth_camera_info` reporting the  intrinsic cameras parameters.


### Physical R4 interface

* The physical hardware interface is based around the R4 open-source hardware board  R4. 


* To establish the communication between the physical hardware board with high level ROS2 stack in order to perform manual teleoperation, Autonomous driving operation via NAV2, UDP protocol is utilized. There are three sets of ROS2 hardware nodes namely; `/R4_Websockets-Clients` which is responsible to establish the connection to transfer the data packets from R4 hardware board to ROS2 over the topic named `/R4` which gets unpacked by the second node `/R4_Publisher` to publish the individual component sub-messages mainly, steer voltage reading, main OSMC motor driver readings over the topics `/R4_AINSTEER` and `/R4_OSMC1`. 


* Following this, these topics are subscribed by ROS2 nodes to control the steering and speed of the vehicle via gamepad for manual teleoperation and via \lstinline{nav2} for autonomous driving tasks.

* The output of either manual teleoperation or `nav2` is same and published over the standard ROS2 message type `Twist` over `/cmd_vel` topic and this information is then subscribed by the third node named as `/R4_Receiver` .


### Simulation

A ROS2 simulation of OpenPodcar2, is provided, using the newly released Gazebo sim. To handle time/clock synchronization issues at the software level, it is necessary that both the systems; ROS2 stack and Gazebo simulation should work on the same time. Although ROS/ROS2 provides the configurable parameter for the nodes named  \lstinline{use_sim_time} which could be set to a boolean value of either true or false, there are still some issues faced namely; lookup transforms, message filter dropping when setting the the whole NAV2 stack with Gazebo which generates lags in the system and ultimately failures. Nevertheless, Navigation2 must be implemented on the real/physical OpenPodcar2, so it makes sense to set the stack with working on wall time/system time. To achieve this condition, the gazebo plugins used in our stack Ackermann Steering plugin which deals the kinematic control for the robot, lidar sensor system plugin to receive the LaserScan data in case of using lidar, rgbd sensor system plugin for simulating the depth image and  odometry publisher plugin to get the ground-truth odometry data out from Gazebo needs to publish the data on wall time and such condition in Gazebo (making use wall time) is not a very discussed topic and in the recently introduced the new Gazebo 7 makes it more of a highly debugging task. To handle this condition, custom ROS2 nodes are created which subscribes to the gazebo output topics and publishes the topic data on wall/system time.


### Packages

1. Pod2_description

This ROS2 package consists the robot's urdf files in the `xacro` directory, meshes of the robot model, sensors in the `meshes` and the `launch` directory contains the `description.launch.py` and  `pod2_description` file which launches the robot model's URDF and the world file in the Gazebo with a condition to start along the rviz2 node.


The launch file also consists the `ros_gz_bridge` package which is used to establish communication between Gazebo and ROS2. The parameter bridge is created for `/model/podcar/cmd_vel` topic from ROS -> GZ, on this topic the Ackermann system plugin publishes the twist messages.
GZ -> ROS is created for  `LaserScan` and `RGBD` based simulated sensor data which is coming from gazebo sensor system plugin, `/model/podcar/odometry` topic consists of ground-truth odometry data from Gazebo.


#### Scripts

This directory in `pod2_description` package consists of intermediate nodes which are used to convert the incoming messages over te topics `/model/podcar/odometry` , LiDAR and RGBD sensor based topics to publish on Wall time. This approach is employed to avoid the time realetd issues, tf errors and with an assumption that the simulation and the physical vehicle should work on same time.


* `odometry_wall_time.py` handles the ground truth odometry `/model/podcar/odometry` topic from GZ and publishes to ROS2 topic `/odom` with changing the time stamp to wall time.


* `laser_wall_time.py` handles the `/lidar_scan` topic from GZ laser plugin and publishes to ROS2 topic `/scan` with changing the time stamp to wall time.


* `RGBD_wall_timer.py` handles the `/rgbd_camera/depth`, `/rgbd_camera/camera_info`, `/rgbd_camera/image`, `rgbd_camera/points` topic from GZ laser plugin and publishes to ROS2 topic `/depth`, `/depth_camera_info`, `/camera/color/image_raw` and `/cloud_in` with changing the time stamp to wall time.


2. Pod2_bringup

* This ROS2 package utilizes the ROS2 teleop-twist-joy pakcage, in order to control the OpenPodcar using the joystick controller in the simulation as well as in real world physical robot teleoperation. 


* Different joystick are tested namely; Logotech Extreme3dPro, generic linux usb joystick, PS2 and XBOX.
To test the joystick is connected to the system run `ls /dev/input`.


* In order to use specific joystick you might have to create the `.yaml` config file which can be referenced from (https://github.com/ros2/teleop_twist_joy/tree/humble/config) and the `launch` directory contains the `joy.launch.py` file which launches the `Joy node` and  `teleop_twist_joy_node`.
**For using any custom joystick, you might need to check which buttons and axis does what** 



I recommend using `https://flathub.org/apps/io.gitlab.jstest_gtk.jstest_gtk`. The tool also provide calibrataion for the joystick which mighht be helpful if deploying on the physical vehicle for teleoperation.

#### Note:

The package also consists physical vehicle control nodes which communicates with low level hardware stack (R4) of OpenPodCar_v2. The credit for developing and testing the hardware stack is given to Mr. Chris Waltham and Dr Charles Fox.
If interested in utilizing the work for Open source hardware project, we strongly recommend to cite  R4 with the paper:


R4: rapid reproducible robotics research open hardware control system https://arxiv.org/abs/2402.09833


3.  Pod2_navigation

Pod2_navigation package consists of the `launch`, `rviz`, `maps`, `config` directories. 

* Config directory:
   This includes the `nav2_dwb_smac.yaml` file which includes the parametrers for AMCL, BT_Navigator, Controller server, PLanner server, Global and Local Costmaps, Behaviour servers, Map server.


*  `mapper_params_slam_sync.yaml` and `mapper_params_slam_async.yaml` are the params file which are used to launch the slam-toolbox either in synchronous/asynchronous mode.


* To run the slam_toolbox for localization, we have to turn off the AMCL and map server. For this, another launch file `navigation.launch.py` is there which launches the required nodes. Now run the slam_toolbox using the launch file.

     `ros2 launch pod2_navigation async_slam.launch.py`.


* If want to build the localize using slam_toolbox, then change the mode to localization in `mapper_params_online_async.yaml`.


* The map can be saved either by command-line: ` ros2 run nav2_map_server map_saver_cli -f <name of the map>` or in rviz2 slam_toolbox plugin. More info could be found at: (https://github.com/SteveMacenski/slam_toolbox/tree/humble).


* In order to save the map with old format (.yaml and .pgm) hit the save map button in rviz2 slam_toolbox plugin and to save in the other format (serilaised), write the name of the map without any extension and  click the serial map button in the rviz2 slamtoolbox_plugin.


* The package has been tested with the `Sim_1.yaml`, `Sim_2.yaml`, `Sim_3.yaml`with corresponding pgm files.


* To use the localization with `slam_toolbox`, you have to provide the right path to the map which you are going to use.  When using slam_toolbox for localization, you do not have to provide the map file extension in the `mapper_params_onlie_async.yaml` and just the name.

#### Note that slam_toolbox is best suited for LiDAR based robots and struggles with RGBD sensor. The OpenPodCar2 features a single RGBD sensor is tested with slam_toolbox with rigorous parametr tuning both in simulation and real physical vehicle. However, due to less angular FOV, the laser scan matching results in sudden jumps of robot. This has been discussed in https://github.com/SteveMacenski/slam_toolbox/issues/662.  To handle this RGBD based slam method RTABMAP is adopted. 



* The launch directory consists of `OpenPodCar_NAV2.launch.py` which uses the default `nav2_bringup` package for launching all the nodes and takes the `parameters from the config directory. It uses AMCL for localization which will also be started.


## III. <a name="bom"></a> Bill of Materials 

| **Name**         | **Component**                                | **USD** | **Source**                                                                                     | **Interface**                     | **Implementation**        |
|-------------------|---------------------------------------------|---------|-------------------------------------------------------------------------------------------------|-----------------------------------|---------------------------|
| Donor vehicle    | Phiseng TE-889XLSN mobility scooter (Branded as Shoprider Traverso) | 6000    | [Shoprider Traverso](https://romamedical.co.uk/shoprider-traveso/)                             | Generic (motor and brake control voltages; mechanical steering linkage) | Patented                 |
| R4               | R4 OSH PCB robot control board              | 300     | [GitHub Repository](https://github.com/orgs/Open-Source-R4-Robotics-Platform/repositories)    | CERN-OHL-W                        | CERN-OHL-W                |
| DepthCam         | Intel RealSense D435                        | 300     | [Intel RealSense D435](https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d435.html) | ROS2 standard                     | Closed                    |
| OSMC             | OSMC motor driver with 24V fan kit          | 230     | [OSMC Motor Driver](https://www.robotpower.com/catalog/)                                     | Generic                           | Public domain             |
| Linear actuator  | Gimson Robotics GLA750-P 12V DC (100mm stroke, 240mm install) | 100     | [Gimson Robotics Actuator](https://gimsonrobotics.co.uk/products/gla-q40-12v-250n-compact-fast-travel-linear-actuator-with-encoder?variant=47116053905684) | Generic                           | Generic                   |
| Laptop stand     | Pyle                                        | 60      | [Laptop Stand](https://uk.redbrain.shop/p/00132017804903)                                     | Generic                           | Generic                   |
| Motor relay      | Ripca 12V, 200A REL-1/H-DUTY/200A/12V       | 40      | [Ripca Relay](https://parts.easycabin.co.uk/products/relay-200a-12volt-heavy-duty)            | Generic                           | Generic                   |
| DBH12            | Dual h-bridge 12V motor driver             | 25      | [Amazon Listing](https://www.amazon.co.uk/Akozon-DC5-12V-0A-30A-Dual-channel-Arduino/dp/B07H2MDXMN) | Generic                           | Closed                    |
| DMH              | Philmore 30-825 SPST Hand Held Push Button Switch | 10      | [Amazon Listing](https://www.amazon.com/Hand-Held-Button-Switch-30-825/dp/B00T6RCGNC)         | Generic                           | Generic                   |
| Fuse20A, Fuse10A | Blade fuses and holders                    | 40      | [RS Components](https://uk.rs-online.com/web/p/fuse-kits/2199556)                            | DIN 72581 standard                | Generic                   |
| 24/19DCDC        | 5A                                         | 20      | [Amazon Listing](https://www.amazon.co.uk/Converter-Waterproof-Regulator-Printers-Surveillance/dp/B087WWTSC4) | Generic                           | Generic                   |
| 24/12DCDC        | 20A                                        | 40      | [Sure Marine Service](https://www.suremarineservice.com/Heat/Converters/DC2412-20C_2.html)   | Generic                           | Generic                   |
| 12/5DCDC         | 10A                                        | 5       | [Amazon Listing](https://www.amazon.co.uk/BuxiuGK-Channel-Optocoupler-Trigger-Expansion/dp/B0BNL4JCKK) | Generic                           | Generic                   |
| DMH relay        | SRD-05VDC-SL-C                             | 13      | [Amazon Listing](https://www.amazon.co.uk/HUAREW-1-channle-optocoupler-isolation-triggering/dp/B0B52RPY43/) | Generic                           | Generic                   |
| Circuit breaker  | DZ47-63 C10                                | 10      | [Amazon Listing](https://www.amazon.co.uk/RKURCK-DZ47-63-Low-voltage-Miniature-Circuit/dp/B0C3CQVWTS) | ISO standard                      | Generic                   |
| Battery charger  | Nexpeak, 10A 12V/24V car battery charger   | 25      | [Amazon Listing](https://www.amazon.co.uk/Automatic-Temperature-Compensation-Motorcycle-Batteries-Red/dp/B094VQ88X2) | Generic                           | Closed                    |
| Perspex board    | 400x200x5mm                                | 10      | [Amazon Listing](https://www.amazon.co.uk/Perspex-Black-Acrylic-Plastic-Choose/dp/B09PRDRGXS/) | Generic                           | Generic                   |
| Nuts, bolts, standoffs | M2-M6 metal and plastic              | ...     | ...                                                                                           | ISO standard                      | Generic                   |
| Wifi router      | Any, 12V powered                          | ...     | ...                                                                                           | IEEE Wifi standard                | Closed                    |
| Connectors       | XT60, 12 pin IDC, 10 pin IDC headers, ribbon cables, Wago connectors, insulated wire, velcro strip, cable ties, heatshrink | ... | ...                                                                                           | Generic                           | Generic                   |



## IV. <a name="software-requirement"></a> Software requirements 


1. Ubuntu 22.04


2. ROS2 Humble full desktop install: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html


3. Gazebo Fortress (Install): https://gazebosim.org/docs/fortress/install


4. The only difference between the Gazebo Fortress and Gazebo Garden is the ros-gz integration package is to build from source for Gazebo Garden while if you are using Gazebo Fortress, the ros_gz package will be installed by binary installation.


5. Gazebo Fortress uses the ignition namepsace when dealing with plugins and setting frame ids for the robot in URDFs or SDFs.

6. Docker installation: https://docs.docker.com/engine/install/ubuntu/ 


## V. <a name="testing-installation"></a> Testing Installation

1. To test that ROS2 is installed properly.
* Open bashrc and add the folowing and save it. `source /opt/ros/humble/setup.bash`.


* Open two terminals, in first run: `ros2 run demo_nodes_cpp talker`, you should see  `hello` in the console.


* In other terminal, run: `ros2 run demo_nodes_py listener`, you should see `I Heard`.


* In order to keep the nodes communication robust, set the `ROS_DOMAIN_ID` in your bashrc. For example: `export ROS_DOMAIN_ID=0`


2. To test the Gazebo Fortress is installed on the system, in the terminal run: `ign gazebo`.If it launches, you'll see the simulation software window.


3. To test the ros_gz package, source the workspace of the package and try the following command:



* `ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@gz.msgs.StringMsg` and view the topic in other terminal using: `ros2 topic list -t`



## VI. <a name="installation-for-openpodcar_v2"></a> Installation for OpenPodCar_V2

To use this package for testing and running simulations using gazebo and ROS2 follow the below instructions:


1. If using Gazebo Fortress, clone this repo following below commands. 



* Make the new workspace, with src directory. `mkdir -p ros2_gz_ws/src`.


* Clone the repository using: `git clone https://github.com/Rak-r/OpenPodCar_V2.git`



2. After cloning the repository, you should have `pod2_description`, `pod2_bringup`, `pod2_navigation`, `pod2_sensor_tools`, `pod2_rtabamap`, `pod2_msgs`in your `src` directory.



3. Now, build the packages from the root of the workspace directory using ROS2 package building tool colcon.


* Assuming you are in /src directory: run `cd ..`


* `colcon build --symlink-install`. This will build the packages and the `--symlink-install` is used to make changes in the packages in src directory and also changes in the install dircetory without re-building the package.



4. If everything works well, you will have three directories alomg with `src` named `install`, `build` ad `log`. If colcon build fails to build any packages and shows `stderr` in terminal console, make sure all the dependencies are install correctly.



5. In case of Step 4, try running: `rosdep update && rosdep install --from-paths src --ignore-src -y`.


6. Once the package is build successfully, open bashrc and add : `source <your workspace path>/install/setup.bash`


* **For example** : `source /home/ros2_ws/install/setup.bash` 

## VII. <a name="docker-support-for-OpenPodcar2"></a> Docker support for OpenPodcar2

The docker version is supported for ROS2 humble and gazebo Fortress due to LTS version of gazebo at the time project development. In future more version suppport will be added. Follow the below instructions for using docker version of OpenPodCar2 with simulation.


1.  After cloning the repository from same above instructions, make sure docker is installed correctly.

2. Install rocker in the local machine to run GUI applications without any hastle inside the container.



` pip3 install rocker`

3. Build the image:     `docker build -t openpodcar2_docker .`

4. Run the container with rocker: `rocker --x11 openpodcar2_docker`.


#### The above will build the container with custom ros2 packages for all tele-operation, mapping and navigation. If want to build the perception stack as well, then build the image with argument in below command.


5. `docker build --build-arg INCLUDE_YOLO=true -t openpodcar2_docker .`

6. Source ros2 and the workspace before running any ros2 nodes or launch files as done in normal local machine setup.

7.  Test the setup by running the below:



`ros2 launch pod2_description pod2_description.launch.py scan_node:=false rgbd_node:=true`

## VIII. <a name="calibration"></a> Calibration

### Depthcam calibration


* The depthcam needs physical calibration in order to achieve reliable SLAM and mapping operation. For this,  place an object 10m away at the same height as the camera from the ground. 


* Adjust camera to ensure that the object appears in the center of the camera image (same height measurement at different distances from the camera). This calibration is essential to verify that the camera is mounted parallel to the ground to avoid irregularities in the mapping which may consider floor as an obstacle.


### Steering calibration


* To calibrate the Ackermann steering angles on both left and right turnings various voltages are sent to the linear actuator and both inner and outer wheel angles are measured. The output result shown that the steering mechanism can be approximated as linear with some fluctuations while small turning angles.


* The mapping between steering angle and desired linear actuator voltage then could be computed using linear regression fit.


## IX. <a name="operator-instructions"></a> Operator intructions

### Simulation
The new Gazebo Garden is used for the simulation of OpenPodCar_v2. The new gazebo features more functionalities with enhanced inetrface. As our robot behaves as car-like robot and features Ackermann-Steering kinematics. To maintain this behaviour in simulation the new gazebo now has an Ackermann system plugin which could be used according the robot configuartions. The plugin outputs standard `Twist` messages of field `linear.x` and `angular.z`. This also outputs the odometry information which might not be the correct odometry for the whole robot instead it is the odometry information for steering.


The current repository features the ROS2 Humble with Gazebo garden. To use the ROS2 Humble packages with Gazebo Fortress, switch to the Fortress branch `https://github.com/Rak-r/OpenPodCar_V2/tree/Fortress`.


#### If you want to launch the PodCar with Lidar enabled, run the below launch file:


* Launch without Rviz : `ros2 launch pod2_decsription pod2_description_Lidar.launch.py scan_node:=true rgbd_node:=false`


* Launch along with Rviz: `ros2 launch pod2_description pod2_description.launch.py rviz:=true scan_node:=true rgbd_node:=false`



#### If you want to launch the PodCar with depth camera enabled, run the below launch file:


* Launch without Rviz : `ros2 launch pod2_decsription pod2_description_Depth.launch.py scan_node:=false rgbd_node:=true`


* Launch along with Rviz: `ros2 launch pod2_description pod2_description_Depth.launch.py rviz:=true scan_node:=false rgbd_node:=true`


This above will launch the simulation in gazebo and don't forget to turn on the play pause button to run the simulation. 
To view the active topics in ros2, use `ros2 topic list -t` in the terminal window.
To view active topics in gazebo, use `gz topic -l` in the terminal window.
[Podcar_V2_GZ_garden.webm](https://github.com/Rak-r/OpenPodCar_/assets/85680564/26ea85f9-a46d-4f53-b81a-1f23425ab1f7)



#### Simulation Teleoperation and Autonomous operation

1. Start gamepad to publish twist to gazebo: `ros2 launch pod2_bringup generic_gamepad.launch.py`

2. Start the rtabmap rgbd odometry and slam: ` ros2 launch pod2_rtabmap rtabmap.launch.py`

3. Launch NAV2 stack: `ros2 launch pod2_navigation OpenPodCar_NAV2.launch.py slam:=false amcl:=false`

After mapping, if want to start the NAV2 stack in pre-build map, rtabmap can be started in localization mode. In order to autonomous drive while mapping the above  could be just followed.



### Physical vehicle Tele-operation & Autonomous operation

To launch the physical OpenPodCar2 with teleoperation mode, the higher-level incoming game-pad commands as Twist message `linear.x, angualr.z` are converted to R4 protocol message which controls the main driver motor for forward and backward movement and linear actuator for controlling the steering for the OpenPodCar2. 

To start the physical vehicle for tele-operation, after building the OpenPodCar2 packaghe from following above instruction.

1. Start the R4-ROS2 communication nodes using the launch file:

`ros2 launch pod2_bringup R4_ros.launch.py teleop_node:=true`

2. Launch the robot model: `ros2 launch pod2_description pod2_description.launch.py`

3. Start the camera sensor along with point to laserscan node: `ros2 launch pod2_sensor_tools point_to_scan.launch.py`

4. Start the rtabmap rgbd odometry and slam: ` ros2 launch pod2_rtabmap rtabmap.launch.py`

5. Launch NAV2 stack: `ros2 launch pod2_navigation OpenPodCar_NAV2.launch.py slam:=false amcl:=false`

After mapping, if want to start the NAV2 stack in pre-build map, rtabmap can be started in localization mode. In order to autonomous drive while mapping the above  could be just followed.


## Images and Videos

### Simulation OpenPodCar_2 with Pedestrian detection and NAV2 operation


https://github.com/user-attachments/assets/e513bdaa-67ae-4721-bcec-ed717356f360


### OpenPodCar_2 on outdoor operation

<p align="center">
  <img src="./Images and videos/Podcar2_outdoor.jpg" width="100%" />
</p>




### Outdoor 3D map built using RTABMAP and single rgbd sensor (intel realsense D435)

<p align="center">
  <img src="./Images and videos/3D_outdoor.png" width="100%" />
</p>



### Autonomous drive tight indoor
   

https://github.com/user-attachments/assets/48aa4406-728c-42f7-9629-9f4f719aca1d


