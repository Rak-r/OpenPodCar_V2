cmake_minimum_required(VERSION 3.8)
project(pod2_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add dependencies for the build tool according to your package.
find_package(ament_cmake REQUIRED)
#This package is cmake build and to support python scripts below is added.
find_package(ament_cmake_python REQUIRED)

#ROS2 APIs
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add the package specific directories in the install directory
install(DIRECTORY launch meshes xacro rviz2 ROS2_Sim_Nodes
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules.
# This is important to provide in order to use python executables
ament_python_install_package(${PROJECT_NAME})

# provide the paths of python scripts for install directory otherwise it will cause the error 'executable not found on libexec directory'

install(PROGRAMS
  ROS2_Sim_Nodes/laser_sim2real.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  ROS2_Sim_Nodes/fixed_sim_twist.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  ROS2_Sim_Nodes/odometry_wall_time.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ROS2_Sim_Nodes/RGBD_wall_timer.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  ROS2_Sim_Nodes/camera_info_modifier.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
