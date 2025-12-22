#!/usr/bin/env python3
# main1_manual_control_real.launch.py

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from hexapod_pkg import hw_config as cfg


def generate_launch_description():

    package_name = "hexapod_pkg"

    # =====================================================
    # BASE DDS
    # =====================================================
    real_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "dds_base.launch.py"
            )
        )
    )

    # =================================================
    # TELEOP TECLADO
    # =================================================
    key_teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop",
        output="screen",
        prefix="xterm -hold -e",
    )


    twist_to_cmd_robot = Node(
        package="hexapod_pkg",
        executable="twist_to_cmd_robot.py",
        name="twist_to_cmd_robot",
        output="screen",
        parameters=[{
            "twist_topic": "/cmd_vel",
            "cmd_robot_topic": cfg.TOPIC_CMD_REAL_ROBOT,
            "deadzone": 0.05,
        }],
    )

    # =================================================
    # COMPUTE
    # =================================================
    compute_heading = Node(
        package=package_name,
        executable="compute_heading.py",
        name="compute_heading",
        output="screen",
        parameters=[{
            "imu_topic": cfg.TOPIC_PI_IMU_GIR_ACC,
            "mag_topic": cfg.TOPIC_PI_IMU_MAG,
            "output_topic": cfg.TOPIC_HEADING_COMPASS,
        }],
    )

    compute_gps_to_local_xy = Node(
        package=package_name,
        executable="compute_gps_to_local_xy.py",
        name="compute_gps_to_local_xy",
        output="screen",
        parameters=[{
            "gps_topic": cfg.TOPIC_PI_GPS,
            "output_topic": "/localization/gps/local_xy",
            "origin_lat_deg": -25.33057356422622,
            "origin_lon_deg": -57.518128840272304,
        }],
    )

    compute_stimate_xy = Node(
        package=package_name,
        executable="compute_stimate_xy.py",
        name="compute_stimate_xy",
        output="screen",
        parameters=[{
            "gps_xy_topic": "/localization/gps/local_xy",
            "heading_topic": cfg.TOPIC_HEADING_COMPASS,
            "hl_cmd_topic": cfg.TOPIC_CMD_REAL_ROBOT,
            "output_topic": "/localization/local_stimate_xy",
            "velocity": 0.06,
            "alpha": 1.0,
            "update_rate": 20.0,
        }],
    )

    # =====================================================
    # LAUNCH
    # =====================================================
    return LaunchDescription([
        real_base,

        # Espera a que DDS esté listo
        TimerAction(
            period=1.0,
            actions=[
                key_teleop,
                twist_to_cmd_robot,
                compute_heading,
                compute_gps_to_local_xy,
                compute_stimate_xy,
            ]
        ),
    ])
