#!/usr/bin/env python3
# main1_manual_control_sim.launch.py

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

    # Define world
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
    # BASE GAZEBO
    # =====================================================
    gazebo_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "gazebo_base.launch.py",
            )
        ),
        launch_arguments={"world": world}.items(),
    )


    # =================================================
    # TELEOP TECLADO
    # =================================================
    key_teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop",
        output="screen",
        prefix=["xterm -hold -e"],
    )

    twist_to_cmd_robot = Node(
        package="hexapod_pkg",
        executable="twist_to_cmd_robot.py",
        name="twist_to_cmd_robot",
        output="screen",
        parameters=[{
            "twist_topic": "/cmd_vel",
            "cmd_robot_topic": cfg.TOPIC_CMD_GZ_ROBOT,
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

        # Espera a que Gazebo esté listo
        TimerAction(
            period=1.0,
            actions=[
                key_teleop,
                twist_to_cmd_robot,
                compute_heading,
                compute_gps_to_local_xy,
                compute_stimate_xy,
            ],
        ),
    ])
