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
    utils
    #launch_sim
    #hexapod_pkg
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

  # Nodos de gazebo
  hexapod_pkg/gz_ir_emulator.py
  hexapod_pkg/gz_hexapod_low_level_control.py

  # Nodos de computo
  hexapod_pkg/compute_gps_to_local_xy.py
  hexapod_pkg/compute_heading.py  #for simulations (by Lucas)
  hexapod_pkg/compute_estimate_xy.py
  hexapod_pkg/compute_ultrasonic_ranges.py

  # Nodos de perifericos
  hexapod_pkg/teleop_hexapod_sim.py
  hexapod_pkg/teleop_hexapod_real.py
  hexapod_pkg/master_monitor.py

  # Nodos de vision
  hexapod_pkg/web_camera_node.py 
  hexapod_pkg/green_cube_viewer2.py 
  hexapod_pkg/yolo_obstacle_node_best.py 
  hexapod_pkg/simple_high_level_nav_with_crab.py 

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

  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
