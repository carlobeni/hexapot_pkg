#!/usr/bin/env python3
# main2_goal_free_navigation_sim.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hexapod_pkg import hw_config as cfg


def generate_launch_description():

    package_name = "hexapod_pkg"
    world = LaunchConfiguration("world")

    # =====================================================
    # WORLD ARG
    # =====================================================
    default_world = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        "obstacles3.world",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world for goal free navigation (simulation)",
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
    # MANAGER DE NAVEGACIÓN A OBJETIVO
    # =====================================================
    manager_goal_free_navigation = Node(
        package="hexapod_pkg",
        executable="manager_goal_free_navigation.py",
        name="manager_goal_free_navigation",
        output="screen",
        parameters=[{
            # ---------- REFERENCIA ----------
            "reference_lat": -25.330480,
            "reference_lon": -57.518124,

            # ---------- OBJETIVO ----------
            "goal_lat": -40.330300,
            "goal_lon": -3.518000,

            # ---------- TOLERANCIAS ----------
            "epsilon": 0.5,          # [m]
            "heading_tol": 6.0,      # [deg]

            # ---------- COMANDOS ----------
            "cmd_robot_topic": cfg.TOPIC_CMD_GZ_ROBOT,
        }],
    )

    # =====================================================
    # COMPUTE (SIM)
    # =====================================================
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
            "output_topic": "/localization/gps/local_xy",
            "origin_lat_deg": -25.330480,
            "origin_lon_deg": -57.518124,
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
            "hl_cmd_topic": "/hl_cmd",
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
        world_arg,
        gazebo_base,

        # Esperar a que Gazebo publique sensores
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
