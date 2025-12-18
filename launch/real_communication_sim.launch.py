#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_name = "hexapod_pkg"

    # =====================================================
    # LIMPIEZA PREVIA
    # =====================================================
    cleanup = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "pkill -f dds_sensors_fast_listener.py || true; "
            "pkill -f dds_sensors_reliable_listener.py || true; "
            "pkill -f dds_cmd_talker.py || true; "
            "pkill -f dds_monitor_pc.py || true; "            
            "pkill -f teleop_twist_keyboard || true; "            
            "pkill -f cmd_vel_relay_node.py || true"
            "pkill -f compute_heading_kalman.py || true"
            
        ],
        output="screen"
    )

    # =====================================================
    # SIMULACIÓN
    # =====================================================
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "rsp.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_ros2_control": "true"
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "joystick.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # (Se mantiene tu twist_mux, aunque ya no es crítico para mover)
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "twist_mux.yaml"
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        output="screen",
    )

    # =====================================================
    # GAZEBO
    # =====================================================
    default_world = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        "green_space.world"
    )

    world = LaunchConfiguration("world")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true"
        }.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "hexapod_pkg",
            "-z", "0.1"
        ],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen",
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
    )

    # =====================================================
    # TELEOP (UNA SOLA TERMINAL)
    # Publica en /cmd_vel (default)
    # =====================================================
    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop",
        output="screen",
        prefix=["xterm -hold -e"],
        # sin remaps: teleop publica /cmd_vel
    )

    # =====================================================
    # RELAY: /cmd_vel  ->  /cmd_vel_robot  y  /diff_cont/cmd_vel_unstamped
    # =====================================================
    cmd_vel_relay = Node(
        package="hexapod_pkg",
        executable="cmd_vel_relay_node.py",
        name="cmd_vel_relay",
        output="screen",
        parameters=[{
            "input_topic": "/cmd_vel",
            "out_robot_topic": "/cmd_vel_robot",
            "out_sim_topic": "/diff_cont/cmd_vel_unstamped",
        }],
    )

    # =====================================================
    # COMUNICACIÓN (CONTROL DEVICE)
    # =====================================================
    sensors_fast_listener = Node(
        package="hexapod_pkg",
        executable="dds_sensors_fast_listener.py",
        name="dds_sensors_fast_listener",
        output="screen",
        parameters=[{
            "qos_reliability": "best_effort",
            "qos_depth": 10,
        }],
    )

    sensors_reliable_listener = Node(
        package="hexapod_pkg",
        executable="dds_sensors_reliable_listener.py",
        name="dds_sensors_reliable_listener",
        output="screen",
        parameters=[{
            "qos_reliability": "reliable",
            "qos_depth": 10,
        }],
    )

    command_talker = Node(
        package="hexapod_pkg",
        executable="dds_cmd_talker.py",
        name="dds_cmd_talker",
        output="screen",
        parameters=[{
            "qos_reliability": "reliable",
            "qos_depth": 10,
        }],
    )

    est_heading_compass = Node(
        package="hexapod_pkg",
        executable="compute_heading_kalman.py",
        name="compute_heading_kalman",
        output="screen",
        parameters=[{
            "qos_reliability": "reliable",
            "qos_depth": 10,
        }],
    )

    monitor = Node(
        package="hexapod_pkg",
        executable="dds_monitor_pc.py",
        name="dds_monitor_pc",
        output="screen",
        prefix=["xterm -hold -e"],
        parameters=[{
            "qos_reliability": "best_effort",
            "qos_depth": 10,
            "monitor_imu_accel": True,
            "monitor_imu_mag": True,
            "monitor_imu_compass": True,
            "monitor_gps": True,
            "monitor_cmd_serial": True,
        }],
    )

    # =====================================================
    # EVENTO HANDLER: LANZAR COMUNICACIÓN CUANDO EL ROBOT YA EXISTE
    # =====================================================
    start_comm_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                sensors_fast_listener,                
                sensors_reliable_listener,
                est_heading_compass,
                command_talker,
                monitor,
                teleop,
                cmd_vel_relay,   # <- IMPORTANTE
            ],
        )
    )

    return LaunchDescription([
        cleanup,

        rsp,
        joystick,
        twist_mux,
        world_arg,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,

        start_comm_after_spawn,
    ])

