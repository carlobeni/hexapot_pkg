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
    # compute_heading = Node(
    #     package=package_name,
    #     executable="compute_heading.py",
    #     name="compute_heading",
    #     output="screen",
    #     parameters=[{
    #         "topic_imu": cfg.TOPIC_GZ_IMU_GIR_ACC,
    #         "topic_mag": cfg.TOPIC_GZ_IMU_MAG,
    #         "topic_estimate_heading": cfg.TOPIC_ESTIMATE_HEADING # publisher
    #     }],
    # )

    compute_gps_to_local_xy = Node(
        package=package_name,
        executable="compute_gps_to_local_xy.py",
        name="compute_gps_to_local_xy",
        output="screen",
        parameters=[{
            "topic_gps_lat_lon_topic": cfg.TOPIC_PI_PHONE_GPS,
            "topic_gps_to_xy": cfg.TOPIC_XY_BY_GPS_CURRENT_POSITION          # publisher
        }],
    )

    compute_estimate_xy = Node(
        package=package_name,
        executable="compute_estimate_xy.py",
        name="compute_estimate_xy",
        output="screen",
        parameters=[{
            "topic_gps_to_xy": cfg.TOPIC_PI_PHONE_GPS,
            "topic_estimate_heading": cfg.TOPIC_HEADING_COMPASS,    
            "topic_estimate_xy": cfg.TOPIC_XY_ODOM_CURRENT_POSITION      
        }],
    )

    compute_ultrasonic_ranges = Node(
        package=package_name,
        executable="compute_ultrasonic_ranges.py",
        name="compute_ultrasonic_ranges",
        output="screen",
        parameters=[{
            "topic_ultrasonic_raw": cfg.TOPIC_PI_ULTRASONIC,
            "topic_ultrasonic_range": cfg.TOPIC_ULTRASONIC_RANGE      # publisher
        }],
    )

    master_monitor = Node(
        package=package_name,
        executable="master_monitor.py",
        name="master_monitor",
        output="screen",
        parameters=[{
            "topic_gps": cfg.TOPIC_PI_PHONE_GPS,
            "topic_imu": cfg.TOPIC_PI_PHONE_IMU_GIR_ACC,
            "topic_ir_left":cfg.TOPIC_PI_IR1,
            "topic_ir_right":cfg.TOPIC_PI_IR2,
            "topic_mag":cfg.TOPIC_PI_PHONE_IMU_MAG,
            "topic_ultrasonic":cfg.TOPIC_PI_ULTRASONIC,
            "topic_gps_to_xy":cfg.TOPIC_XY_BY_GPS_CURRENT_POSITION,
            "topic_estimate_heading":cfg.TOPIC_XY_ODOM_CURRENT_POSITION,
            "topic_ultrasonic_range":cfg.TOPIC_ULTRASONIC_RANGE
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
                compute_heading,
                compute_gps_to_local_xy,
                compute_estimate_xy,
                compute_ultrasonic_ranges,
                master_monitor
            ]
        ),
    ])
