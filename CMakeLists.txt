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
    launch
    worlds
    meshes
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
  # Nodos de comunicacion
  hexapod_pkg/dds_cmd_talker.py
  hexapod_pkg/dds_monitor_pc.py
  hexapod_pkg/dds_sensors_fast_listener.py
  hexapod_pkg/dds_sensors_reliable_listener.py

  # Nodos utils
  #hexapod_pkg/utils/cmd_vel_relay_node.py # nodo para difusion de robot+gazebo
  #hexapod_pkg/utils/twist_to_cmd_robot.py

  # Files utils
  #hexapod_pkg/utils/test2.py
  #hexapod_pkg/utils/hand_landmarker.task
  #hexapod_pkg/utils/best.pt
  #hexapod_pkg/utils/best_custom.pt


  # Nodos de gazebo
  hexapod_pkg/gz_ir_emulator.py
  hexapod_pkg/gz_hexapod_low_level_control.py #(add LPv1) 

  # Nodos de gestion de sensores
  hexapod_pkg/sensor_ultrasonic.py

  # Nodos de computo
  hexapod_pkg/compute_gps_to_local_xy.py
  hexapod_pkg/compute_heading.py  #for simulations (by Lucas)
  hexapod_pkg/compute_heading_kalman.py #by for implementation (by Carlos)
  hexapod_pkg/compute_stimate_xy.py

  # Nodos de vision 
  hexapod_pkg/teleop_hexapod.py #(add LPv1)
  hexapod_pkg/web_camera_node.py #(add LPv1)
  hexapod_pkg/green_cube_viewer2.py #(add LPv1)
  hexapod_pkg/yolo_obstacle_node_best.py #(add LPv1)
  #hexapod_pkg/best_custom.pt #(add LPv1)

  # Nodos manager

  # Modo 1: manual_control
  # SIN MANAGER

  # Modo 2: goal_free_navigation
  hexapod_pkg/manager_goal_free_navigation.py

  # Modo 3: goal_cube_navigation
  #hexapod_pkg/manager_goal_cube_navigation.py

  # Modo 4: follow_target_real
  #hexapod_pkg/manager_follow_target_real.py

  # Modo 5: swarm_aggregation_manager.py
  #hexapod_pkg/manager_swarm_aggregation_manager.py

  #hexapod_pkg/utils/cmd_vel_relay_node.py
  #hexapod_pkg/test2.py
  #hexapod_pkg/localization_nodes/gps_to_local_xy.py
  #hexapod_pkg/localization_nodes/compute_heading.py
  #hexapod_pkg/navigation_nodes/simple_high_level_nav.py
  #hexapod_pkg/navigation_nodes/high_level_A_planner.py
  #hexapod_pkg/hexapod_movements_nodes/hexapod_low_level_control.py
  #hexapod_pkg/localization_nodes/corrected_stimate_xy.py
  #hexapod_pkg/navigation_nodes/local_nav_planner.py
  #hexapod_pkg/image_recognition_nodes/yolo_obstacle_node.py
  #hexapod_pkg/social_robot_nodes/hand_gesture_node.py
  #hexapod_pkg/hand_landmarker.task
  #hexapod_pkg/social_robot_nodes/social_pose_node.py
  #hexapod_pkg/best.pt
  #hexapod_pkg/image_recognition_nodes/obstacle_viewer.py
  #hexapod_pkg/navigation_nodes/local_nav_planner2.py
  #hexapod_pkg/navigation_nodes/global_nav_planner.py
  #hexapod_pkg/collision_detector_nodes/ultrasonic_range.py
  #hexapod_pkg/hexapod_movements_nodes/teleop_hexapod.py
  #hexapod_pkg/simulation_utils/ir_gazebo_ros_emulator.py 
  #hexapod_pkg/master_nodes/master_monitor.py
  #worlds/textura-del-asfalto.jpg
  #hexapod_pkg/master_nodes/control_master_hexapod.py 
  #hexapod_pkg/localization_nodes/noise_compute_heading.py
  #hexapod_pkg/localization_nodes/noise_gps_to_local_xy.py 

  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
