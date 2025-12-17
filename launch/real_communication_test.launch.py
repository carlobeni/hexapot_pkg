#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():

    package_name = "hexapot_pkg"

    # =====================================================
    # LIMPIEZA PREVIA
    # =====================================================
    cleanup = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "pkill -f sensors_fast_listener_node.py || true; "
            "pkill -f sensors_reliable_listener_node.py || true; "
            "pkill -f command_talker_node.py || true; "
            "pkill -f monitor_node_pc.py || true; "
            "pkill -f heading_estimator_node.py || true; "
            "pkill -f twist_mux || true"
        ],
        output="screen"
    )

    # =====================================================
    # TWIST_MUX
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
        # 🔑 salida estándar del mux
        remappings=[
            ("/cmd_vel_out", "/cmd_vel_robot")
        ],
    )

    # =====================================================
    # LAUNCH
    # =====================================================
    return LaunchDescription([

        # ===== LIMPIEZA =====
        cleanup,

        # Espera corta (mantienes tu enfoque)
        TimerAction(
            period=1.0,
            actions=[

                # ===== TWIST MUX =====
                twist_mux,

                # ===== FAST LISTENER =====
                Node(
                    package="hexapot_pkg",
                    executable="sensors_fast_listener_node.py",
                    name="sensors_fast_listener",
                    output="screen",
                    parameters=[{
                        "qos_reliability": "best_effort",
                        "qos_history": "keep_last",
                        "qos_depth": 10,
                    }],
                ),

                # ===== RELIABLE LISTENER =====
                Node(
                    package="hexapot_pkg",
                    executable="sensors_reliable_listener_node.py",
                    name="sensors_reliable_listener",
                    output="screen",
                    parameters=[{
                        "qos_reliability": "reliable",
                        "qos_history": "keep_last",
                        "qos_depth": 10,
                    }],
                ),

                # ===== COMMAND TALKER =====
                Node(
                    package="hexapot_pkg",
                    executable="command_talker_node.py",
                    name="command_talker",
                    output="screen",
                    parameters=[{
                        "qos_reliability": "reliable",
                        "qos_history": "keep_last",
                        "qos_depth": 10,
                    }],
                ),

                # ===== KEYBOARD TELEOP =====
                Node(
                    package="teleop_twist_keyboard",
                    executable="teleop_twist_keyboard",
                    name="teleop",
                    output="screen",
                    prefix=["xterm -hold -e"],
                    # sin remaps: teleop publica /cmd_vel
                ),

                # =====================================================
                # RELAY: /cmd_vel  ->  /cmd_vel_robot  y  /diff_cont/cmd_vel_unstamped
                # (ESTO ES LO QUE TE FALTABA: copiar datos realmente)
                # =====================================================
                Node(
                    package="hexapot_pkg",
                    executable="cmd_vel_relay_node.py",
                    name="cmd_vel_relay",
                    output="screen",
                    parameters=[{
                        "input_topic": "/cmd_vel",
                        "out_robot_topic": "/cmd_vel_robot",
                        #"out_sim_topic": "/diff_cont/cmd_vel_unstamped",
                    }],
                ),

                # ===== MONITOR =====
                Node(
                    package="hexapot_pkg",
                    executable="monitor_node_pc.py",
                    name="monitor_node_pc",
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
