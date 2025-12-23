#!/usr/bin/env python3
# main1_manual_control_real.launch.py

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from hexapod_pkg import hw_config as cfg

# AGREGAR PARAMETROS PARA CAMBIAR TOPICS AL LANZAR CON PARAMETROS POR DEFECTO DE GZ

def generate_launch_description():

    package_name = "hexapod_pkg"

    # =================================================
    # COMPUTE
    # =================================================
    cube_viewer_classic_vission = Node(
        package=package_name,
        executable="cube_viewer_classic_vission.py",
        name="cube_viewer_classic_vission",
        output="screen",
        parameters=[{
            "topic_image": cfg.TOPIC_GZ_CAMERA,
            "topic_obstacle_occupation_grid": cfg.TOPIC_OBSTACLE_OCCUPATION_GRID  # publisher
        }],
    )

    cube_viewer_yolo = Node(
        package=package_name,
        executable="cube_viewer_yolo.py",
        name="cube_viewer_yolo",
        output="screen",
        parameters=[{
            "topic_image": cfg.TOPIC_GZ_CAMERA,
            "topic_obstacle_occupation_grid": cfg.TOPIC_OBSTACLE_OCCUPATION_GRID  # publisher
        }],
    )

    target_navigation_with_obstacle_avoidance = Node(
        package=package_name,
        executable="target_navigation_with_obstacle_avoidance.py",
        name="target_navigation_with_obstacle_avoidance",
        output="screen",
        parameters=[{
            "topic_ir_left": cfg.TOPIC_GZ_GPS,
            "topic_ir_right": cfg.TOPIC_XY_BY_GPS_CURRENT_POSITION,          
            "topic_estimate_heading":cfg.TOPIC_HEADING_COMPASS,
            "topic_ultrasonic_range": cfg.TOPIC_ULTRASONIC_RANGE,
            "topic_estimate_xy":cfg.TOPIC_XY_ODOM_CURRENT_POSITION,
            "topic_cmd_robot": cfg.TOPIC_CMD_GZ_ROBOT,
            "topic_obstacle_occupation_grid": cfg.TOPIC_OBSTACLE_OCCUPATION_GRID,
            "LAT0": cfg.LAT_REFERENCE,
            "LON0": cfg.LON_REFERENCE
        }],
    )

    # =====================================================
    # LAUNCH
    # =====================================================
    return LaunchDescription([
        # Espera a que DDS esté listo
        TimerAction(
            period=1.0,
            actions=[
                cube_viewer_yolo,
                target_navigation_with_obstacle_avoidance
            ]
        ),
    ])
