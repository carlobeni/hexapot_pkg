#!/usr/bin/env python3
# gazebo_base.launch.py

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
    # ARGUMENTOS
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
        description="Gazebo world file",
    )

    # =====================================================
    # ROBOT STATE
    # =====================================================
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "rsp.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_ros2_control": "true",
        }.items(),
    )

    # =====================================================
    # GAZEBO ENVIRONMENT
    # =====================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "hexapod",
            "-z", "0.02",
        ],
        output="screen",
    )

    # =====================================================
    # ROS2 CONTROL
    # =====================================================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    # =====================================================
    # GAZEBO ROS BRIDGES
    # =====================================================
    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "gz_bridge.yaml",
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
        output="screen",
    )

    # =====================================================
    # SENSORES SIMULADOS
    # =====================================================
    gz_ir_emulator = Node(
        package="hexapod_pkg",
        executable="gz_ir_emulator.py",
        name="gz_ir_emulator",
        output="screen",
    )

    # =====================================================
    # CINEMÁTICA INVERSA
    # OBS: este nodo aguarda que se publique en cfg.TOPIC_CMD_GZ_ROBOT para mover el robot
    # =====================================================
    gz_hexapod_inverse_kinematics = Node(
        package="hexapod_pkg",
        executable="gz_hexapod_inv_kinematics.py",
        name="gz_hexapod_inv_kinematics",
        output="screen",
    )

    # =====================================================
    # LAUNCH
    # =====================================================
    return LaunchDescription(
        [
            world_arg,
            rsp,
            gazebo,
            spawn_entity,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
            ros_gz_bridge,
            ros_gz_image_bridge,
            gz_ir_emulator,
            gz_hexapod_inverse_kinematics,
        ]
    )
