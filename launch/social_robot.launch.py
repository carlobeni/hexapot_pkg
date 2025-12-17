#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = "hexapot_pkg"

    # ==========================================================
    # ARGUMENTOS
    # ==========================================================
    image_width  = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    camera_frame = LaunchConfiguration("camera_frame")

    declare_image_width = DeclareLaunchArgument(
        "image_width",
        default_value="640",
        description="Camera image width"
    )

    declare_image_height = DeclareLaunchArgument(
        "image_height",
        default_value="480",
        description="Camera image height"
    )

    declare_camera_frame = DeclareLaunchArgument(
        "camera_frame",
        default_value="camera_optical_link",
        description="Camera optical frame"
    )

    # ==========================================================
    # CÁMARA (v4l2)
    # ==========================================================
    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        output="screen",
        parameters=[{
            "image_size": [640, 480],
            "camera_frame_id": "camera_optical_link",
            "device": "/dev/video0"
        }]
    )
    # ==========================================================
    # NODO DE GESTOS
    # ==========================================================
    hand_gesture_node = Node(
        package=pkg_name,
        executable="hand_gesture_node.py",
        name="hand_gesture_node",
        output="screen",
        remappings=[
            ("/image_raw", "/image_raw")  # explícito, por claridad
        ]
    )

    # ==========================================================
    # NODO DE POSES SOCIALES
    # ==========================================================
    social_motion_node = Node(
        package=pkg_name,
        executable="social_pose_node.py",
        name="social_motion_node",
        output="screen"
    )

    # ==========================================================
    # LAUNCH
    # ==========================================================
    return LaunchDescription([
        declare_image_width,
        declare_image_height,
        declare_camera_frame,

        camera_node,
        hand_gesture_node,
        social_motion_node,
    ])
