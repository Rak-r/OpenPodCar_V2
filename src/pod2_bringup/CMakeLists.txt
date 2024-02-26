cmake_minimum_required(VERSION 3.8)
project(pod2_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add dependencies for the build tool according to your package.
find_package(ament_cmake REQUIRED)
#This package is cmake build and to support python scripts below is added.
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# Add the package specific directories in the install directory
install(DIRECTORY launch config ROS2_R4_nodes
  DESTINATION share/${PROJECT_NAME}/
)
# Install Python modules.
# This is important to provide in order to use python executables
ament_python_install_package(${PROJECT_NAME})

# provide the paths of python scripts for install directory otherwise it will cause the error 'executable not found on libexec directory'


########### R4 specific Nodes to communicate with ROS2################################################################ 
###########and Low level motor control ROS2 nodes which takes in cmd_vel and converts to R4_Protocol messages ########

install(PROGRAMS
ROS2_R4_nodes/Podcar_motor_driver.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
ROS2_R4_nodes/Podcar_Steer_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
ROS2_R4_nodes/R4_Publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
ROS2_R4_nodes/R4_Receiver.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
ROS2_R4_nodes/R4-Websockets-Client.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
ROS2_R4_nodes/Config.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
ROS2_R4_nodes/supportFunctions.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
