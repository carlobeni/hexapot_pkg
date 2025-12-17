cmake_minimum_required(VERSION 3.5)
project(hexapot_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# =====================================================
# Dependencies (Python ROS 2)
# =====================================================
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# =====================================================
# Testing 
# =====================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =====================================================
# Install config, description, launch, worlds
# =====================================================
install(
  DIRECTORY 
    config 
    description 
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# =====================================================
# Install hw_config.py
# =====================================================
install(
  FILES hexapot_pkg/communication/hw_config.py
  DESTINATION lib/${PROJECT_NAME}
)

# =====================================================
# Install Python nodes (EXACT FILENAMES)
# =====================================================
install(PROGRAMS
  # Nodos de comunicacion
  hexapot_pkg/communication/sensors_fast_listener_node.py
  hexapot_pkg/communication/sensors_reliable_listener_node.py
  hexapot_pkg/communication/command_talker_node.py
  hexapot_pkg/communication/monitor_node_pc.py

  # Nodos manager

  # Nodos utils
  hexapot_pkg/utils/cmd_vel_relay_node.py # nodo para difusion de robot+gazebo
  hexapot_pkg/communication/heading_estimator_node.py # nodo para estimation de heading
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
