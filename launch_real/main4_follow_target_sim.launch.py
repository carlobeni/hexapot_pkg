#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_name = "hexapod_pkg"

    # =====================================================
    # ARGUMENTOS DEL LAUNCH PRINCIPAL
    # =====================================================
    world = LaunchConfiguration("world")

    default_world = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        "obstacles3.world",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world for follow_target simulation",
    )

    # =====================================================
    # GAZEBO BASE (SIMULACIÓN)
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
    # NAVIGATION MANAGER
    # =====================================================
    follow_target_manager = Node(
        package=package_name,
        executable="hexapod_navigation_manager.py",
        name="follow_target_manager",
        output="screen",
        parameters=[{
            "mode": "follow_target",
            "environment": "sim",

            # Parámetros específicos del modo
            "target_robot_id": 1,          # ID del robot a seguir
            "desired_distance": 1.5,       # distancia objetivo [m]
        }],
    )

    # =====================================================
    # LAUNCH DESCRIPTION
    # =====================================================
    return LaunchDescription([
        world_arg,
        gazebo_base,
        follow_target_manager,
    ])
