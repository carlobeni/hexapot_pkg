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
    description_hexapod
    launch
    worlds
    meshes
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

  # Nodos utils
  hexapot_pkg/utils/cmd_vel_relay_node.py # nodo para difusion de robot+gazebo
  hexapot_pkg/communication/heading_estimator_node.py # nodo para estimation de heading

  # Files utils
  hexapot_pkg/utils/test2.py
  hexapot_pkg/utils/hand_landmarker.task
  hexapot_pkg/utils/best.pt

  # Nodes
  hexapot_pkg/utils/cmd_vel_relay_node.py
  #hexapot_pkg/test2.py
  hexapot_pkg/localization_nodes/gps_to_local_xy.py
  hexapot_pkg/localization_nodes/compute_heading.py
  hexapot_pkg/navigation_nodes/simple_high_level_nav.py
  hexapot_pkg/navigation_nodes/high_level_A_planner.py
  hexapot_pkg/hexapod_movements_nodes/hexapod_low_level_control.py
  hexapot_pkg/localization_nodes/corrected_stimate_xy.py
  hexapot_pkg/navigation_nodes/local_nav_planner.py
  hexapot_pkg/image_recognition_nodes/yolo_obstacle_node.py
  hexapot_pkg/social_robot_nodes/hand_gesture_node.py
  #hexapot_pkg/hand_landmarker.task
  hexapot_pkg/social_robot_nodes/social_pose_node.py
  #hexapot_pkg/best.pt
  hexapot_pkg/image_recognition_nodes/obstacle_viewer.py
  hexapot_pkg/navigation_nodes/local_nav_planner2.py
  hexapot_pkg/navigation_nodes/global_nav_planner.py
  hexapot_pkg/collision_detector_nodes/ultrasonic_range.py
  hexapot_pkg/hexapod_movements_nodes/teleop_hexapod.py
  hexapot_pkg/simulation_utils/ir_gazebo_ros_emulator.py 
  hexapot_pkg/master_nodes/master_monitor.py
  #worlds/textura-del-asfalto.jpg
  hexapot_pkg/master_nodes/control_master_hexapod.py 
  hexapot_pkg/localization_nodes/noise_compute_heading.py
  hexapot_pkg/localization_nodes/noise_gps_to_local_xy.py 

  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
