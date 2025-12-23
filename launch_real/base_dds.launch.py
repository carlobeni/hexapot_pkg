#!/usr/bin/env python3
# dds_base.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

from hexapod_pkg import hw_config as cfg


def generate_launch_description():

    package_name = "hexapod_pkg"

    # =====================================================
    # LIMPIEZA PREVIA (solo nodos de esta base)
    # =====================================================
    cleanup = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "pkill -f dds_sensors_fast_listener.py || true; "
            "pkill -f dds_sensors_reliable_listener.py || true; "
            "pkill -f dds_cmd_talker.py || true"
        ],
        output="screen"
    )

    return LaunchDescription([
        cleanup,

        # -------- ESPERA CORTA PARA DDS --------
        TimerAction(
            period=1.0,
            actions=[

                # =================================================
                # DDS FAST LISTENER (sensores rápidos)
                # =================================================
                Node(
                    package="hexapod_pkg",
                    executable="dds_sensors_fast_listener.py",
                    name="dds_sensors_fast_listener",
                    output="screen",
                    parameters=[{
                        "imu_topic": cfg.TOPIC_PI_PHONE_IMU_GIR_ACC,
                        "mag_topic": cfg.TOPIC_PI_PHONE_IMU_MAG,
                        "ultrasonic_topic": cfg.TOPIC_PI_ULTRASONIC,
                        "ir1_topic": cfg.TOPIC_PI_IR1,
                        "ir2_topic": cfg.TOPIC_PI_IR2,
                        "camera_topic": cfg.TOPIC_PI_PHONE_CAMERA,
                    }],
                ),

                # =================================================
                # DDS RELIABLE LISTENER (sensores críticos)
                # =================================================
                Node(
                    package="hexapod_pkg",
                    executable="dds_sensors_reliable_listener.py",
                    name="dds_sensors_reliable_listener",
                    output="screen",
                    parameters=[{
                        "gps_topic": cfg.TOPIC_PI_PHONE_GPS,
                    }],
                ),

                # =================================================
                # DDS COMMAND TALKER (cmd_robot → cmd_serial)
                # =================================================
                Node(
                    package="hexapod_pkg",
                    executable="dds_cmd_talker.py",
                    name="dds_cmd_talker",
                    output="screen",
                    parameters=[{
                        "cmd_robot_topic": cfg.TOPIC_CMD_REAL_ROBOT,
                        "cmd_serial_topic": cfg.TOPIC_CMD_SERIAL,
                        "linear_speed": 127,
                        "angular_speed": 127,
                        "walk_yaw_trim": -7,
                    }],
                ),

                # # =================================================
                # # DDS MONITOR PC
                # # =================================================
                # Node(
                #     package="hexapod_pkg",
                #     executable="dds_monitor_pc.py",
                #     name="dds_monitor_pc",
                #     output="screen",
                #     prefix=["xterm -hold -e"],
                #     parameters=[{
                #         "topic_imu": cfg.TOPIC_PI_IMU_GIR_ACC,
                #         "topic_mag": cfg.TOPIC_PI_IMU_MAG,
                #         "topic_gps": cfg.TOPIC_PI_GPS,
                #         "topic_ultrasonic": cfg.TOPIC_PI_ULTRASONIC,
                #         "topic_ir1": cfg.TOPIC_PI_IR1,
                #         "topic_ir2": cfg.TOPIC_PI_IR2,
                #         "topic_camera": cfg.TOPIC_PI_CAMERA,
                #         "topic_cmd_serial": cfg.TOPIC_CMD_SERIAL,
                #     }],
                # ),
            ]
        ),
    ])
