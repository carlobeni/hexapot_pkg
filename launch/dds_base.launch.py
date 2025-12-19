#!/usr/bin/env python3
# dds_base.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():

    package_name = "hexapod_pkg"

    # =====================================================
    # LIMPIEZA PREVIA (EVITA NODOS DUPLICADOS)
    # =====================================================
    cleanup = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "pkill -f dds_sensors_fast_listener.py || true; "
            "pkill -f dds_sensors_reliable_listener.py || true; "
            "pkill -f dds_cmd_talker.py || true; "
            "pkill -f dds_monitor_pc.py || true; "
            "pkill -f twist_mux || true"
        ],
        output="screen"
    )

    # =====================================================
    # TWIST_MUX (ARBÍTRO DE VELOCIDADES)
    # =====================================================
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "twist_mux.yaml"
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            twist_mux_params,
            {"use_sim_time": False}
        ],
        remappings=[
            ("/cmd_vel_out", "/cmd_vel_robot")
        ],
    )

    # =====================================================
    # LAUNCH DESCRIPTION
    # =====================================================
    return LaunchDescription([

        # -------- LIMPIEZA --------
        cleanup,

        # -------- ESPERA CORTA --------
        TimerAction(
            period=1.0,
            actions=[

                # ===== TWIST MUX =====
                twist_mux,

                # ===== DDS FAST LISTENER =====
                Node(
                    package="hexapod_pkg",
                    executable="dds_sensors_fast_listener.py",
                    name="dds_sensors_fast_listener",
                    output="screen",
                    parameters=[{
                        "qos_reliability": "best_effort",
                        "qos_history": "keep_last",
                        "qos_depth": 10,
                    }],
                ),

                # ===== DDS RELIABLE LISTENER =====
                Node(
                    package="hexapod_pkg",
                    executable="dds_sensors_reliable_listener.py",
                    name="dds_sensors_reliable_listener",
                    output="screen",
                    parameters=[{
                        "qos_reliability": "reliable",
                        "qos_history": "keep_last",
                        "qos_depth": 10,
                    }],
                ),

                # ===== DDS COMMAND TALKER =====
                Node(
                    package="hexapod_pkg",
                    executable="dds_cmd_talker.py",
                    name="dds_cmd_talker",
                    output="screen",
                    parameters=[{
                        "qos_reliability": "reliable",
                        "qos_history": "keep_last",
                        "qos_depth": 10,
                    }],
                ),

                # ===== TELEOP TECLADO =====
                Node(
                    package="teleop_twist_keyboard",
                    executable="teleop_twist_keyboard",
                    name="teleop",
                    output="screen",
                    prefix=["xterm -hold -e"],
                ),

                # =====================================================
                # CMD_VEL RELAY
                # /cmd_vel → /cmd_vel_robot
                # =====================================================
                Node(
                    package="hexapod_pkg",
                    executable="cmd_vel_relay_node.py",
                    name="cmd_vel_relay",
                    output="screen",
                    parameters=[{
                        "input_topic": "/cmd_vel",
                        "out_robot_topic": "/cmd_vel_robot",
                    }],
                ),

                # ===== MONITOR PC =====
                Node(
                    package="hexapod_pkg",
                    executable="dds_monitor_pc.py",
                    name="dds_monitor_pc",
                    output="screen",
                    prefix=["xterm -hold -e"],
                    parameters=[{
                        "qos_reliability": "best_effort",
                        "qos_history": "keep_last",
                        "qos_depth": 10,

                        "monitor_imu_accel": True,
                        "monitor_imu_mag": True,
                        "monitor_imu_compass": True,
                        "monitor_gps": True,
                        "monitor_cmd_serial": True,
                    }],
                ),
            ]
        ),
    ])
