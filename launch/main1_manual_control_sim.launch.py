#!/usr/bin/env python3
# main1_manual_control_sim.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from hexapod_pkg import hw_config as cfg

def generate_launch_description():

    package_name = "hexapod_pkg"
    world = LaunchConfiguration("world")

    default_world = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        "obstacles3.world",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world for manual control (simulation)",
    )

    # =====================================================
    # GAZEBO BASE
    # =====================================================
    gazebo_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "gazebo_base.launch.py",
            )
        ),
        launch_arguments={
            "world": world
        }.items(),
    )

    # =====================================================
    # NODOS DE COMPUTO (USAN TOPICS GZ)
    # =====================================================

    compute_heading = Node(
        package=package_name,
        executable="compute_heading.py",
        name="compute_heading",
        output="screen",
        parameters=[{
            "imu_topic": cfg.TOPIC_GZ_IMU_GIR_ACC,
            "mag_topic": cfg.TOPIC_GZ_IMU_MAG,
        }],
    )

    compute_gps_to_local_xy = Node(
        package="hexapod_pkg",
        executable="compute_heading.py",
        name="compute_heading",
        output="screen",
        parameters=[{
            "imu_topic": cfg.TOPIC_GZ_IMU_GIR_ACC,
            "mag_topic": cfg.TOPIC_GZ_IMU_MAG,
            "output_topic": cfg.TOPIC_HEADING_COMPASS,

            "declination_deg": -15.24,
            "alpha": 0.95,
            "heading_gain": 1.12,
        }],
    )

    compute_stimate_xy = Node(
        package="hexapod_pkg",
        executable="compute_stimate_xy.py",
        name="compute_stimate_xy",
        output="screen",
        parameters=[{
            "gps_xy_topic": "/localization/gps/local_xy",
            "heading_topic": cfg.TOPIC_HEADING_COMPASS,
            "hl_cmd_topic": "/hl_cmd", #para inverse kinematics
            "output_topic": "/localization/local_stimate_xy",

            "velocity": 0.06,
            "alpha": 0.9,
            "update_rate": 20.0,
        }],
    )

    # =====================================================
    # LAUNCH
    # =====================================================
    return LaunchDescription([
        world_arg,
        gazebo_base,
        compute_heading,
        compute_gps_to_local_xy,
        compute_stimate_xy,
    ])
