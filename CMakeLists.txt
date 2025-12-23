cmake_minimum_required(VERSION 3.5)
project(hexapod_pkg)

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
ament_python_install_package(${PROJECT_NAME})


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
    description_hexapod
    worlds
    meshes
    utils
    launch_sim
    launch_real 
  DESTINATION share/${PROJECT_NAME}
)

# =====================================================
# Install hw_config.py
# =====================================================
install(
  FILES hexapod_pkg/hw_config.py
  DESTINATION lib/${PROJECT_NAME}
)

# =====================================================
# Install Python nodes (EXACT FILENAMES)
# =====================================================
install(PROGRAMS

  # Nodos de la camara
  hexapod_pkg/camera_nodes/web_camera_node.py

  # Nodos dds
  hexapod_pkg/dds_nodes/dds_cmd_talker.py
  hexapod_pkg/dds_nodes/dds_monitor_pc.py
  hexapod_pkg/dds_nodes/dds_sensors_fast_listener.py
  hexapod_pkg/dds_nodes/dds_sensors_reliable_listener.py

  # Nodos gz
  hexapod_pkg/gazebo_nodes/gz_ir_emulator.py
  hexapod_pkg/gazebo_nodes/gz_hexapod_low_level_control.py

  # Nodos location and orientation
  hexapod_pkg/localization_and_orientation_nodes/compute_gps_to_local_xy.py
  hexapod_pkg/localization_and_orientation_nodes/compute_heading.py 
  hexapod_pkg/localization_and_orientation_nodes/compute_estimate_xy.py

  # Nodos sensor
  hexapod_pkg/sensors_interfaces_nodes/compute_ultrasonic_ranges.py

  # Nodos monitor
  hexapod_pkg/monitor_nodes/master_monitor.py

  # Nodos Image Recognition
  hexapod_pkg/image_recognition_nodes/cube_viewer_classic_vision.py
  hexapod_pkg/image_recognition_nodes/cube_viewer_yolo.py
  hexapod_pkg/image_recognition_nodes/yolo_obstacle_node_best.py 
  hexapod_pkg/image_recognition_nodes/ball_viewer_yolo.py 

  # Nodos TeleOP
  hexapod_pkg/teleop_nodes/teleop_hexapod.py

  # Nodos Navigation
  hexapod_pkg/navigation_nodes/target_navigation_with_obstacle_avoidance.py
  hexapod_pkg/navigation_nodes/follower_navigation_node.py 

  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
