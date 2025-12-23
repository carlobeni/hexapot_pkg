#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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

    manager_goal_free_navigation = Node(
        package="hexapod_pkg",
        executable="manager_goal_free_navigation.py",
        name="manager_goal_free_navigation",
        output="screen",
        parameters=[{
            "current_position_xy": cfg.TOPIC_XY_ODM_CURRENT_POSITION,
            "heading_deg_topic": cfg.TOPIC_HEADING_COMPASS,
            # Referencia (esquina de la cancha): -25.33057356422622, -57.518128840272304
            "reference_lat": -25.33057356422622,
            "reference_lon": -57.518128840272304,
            #Centro de la cancha de bascket -25.330587999442308, -57.51791576777312
            "goal_lat": -25.330587999442308,
            "goal_lon": -57.51791576777312,

            "epsilon": 1.0,
            "heading_tol": 6.0,

            "time_forward": 1.2,
            "cmd_robot_topic": cfg.TOPIC_CMD_GZ_ROBOT,
        }],
    )

    compute_heading = Node(
        package=package_name,
        executable="compute_heading.py",
        name="compute_heading",
        output="screen",
        parameters=[{
            "imu_topic": cfg.TOPIC_GZ_IMU_GIR_ACC,
            "mag_topic": cfg.TOPIC_GZ_IMU_MAG,
            "output_topic": cfg.TOPIC_HEADING_COMPASS,
        }],
    )

    compute_gps_to_local_xy = Node(
        package=package_name,
        executable="compute_gps_to_local_xy.py",
        name="compute_gps_to_local_xy",
        output="screen",
        parameters=[{
            "gps_topic": cfg.TOPIC_GZ_GPS,
            "output_topic": cfg.TOPIC_XY_BY_GPS_CURRENT_POSITION,
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
            "gps_xy_topic": cfg.TOPIC_XY_BY_GPS_CURRENT_POSITION,
            "heading_topic": cfg.TOPIC_HEADING_COMPASS,
            "hl_cmd_topic": "/hl_cmd",
            "output_topic": cfg.TOPIC_XY_ODM_CURRENT_POSITION,

            "fixed_dt": 0.1,
            "velocity": 0.02,

            "alpha": 0.9,
            "update_rate": 10.0,
        }],
    )

    return LaunchDescription([
        real_base,

        TimerAction(
            period=1.0,
            actions=[
                compute_heading,
                compute_gps_to_local_xy,
                compute_stimate_xy,
                manager_goal_free_navigation,
            ],
        ),
    ])
