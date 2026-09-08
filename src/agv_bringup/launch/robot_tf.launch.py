#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AutoFactory Waffle TF bringup.

功能：
1. 启动 TurtleBot3 Waffle 官方 robot_state_publisher；
2. 保持现有机器人 TF 树；
3. 永久发布：
   camera_rgb_frame -> camera_rgb_optical_frame

ROS常规机器人坐标系：
    +X 前
    +Y 左
    +Z 上

ROS标准相机光学坐标系：
    +X 右
    +Y 下
    +Z 前
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Create AutoFactory robot TF launch description."""

    use_sim_time = LaunchConfiguration("use_sim_time")

    turtlebot3_gazebo_share = get_package_share_directory(
        "turtlebot3_gazebo"
    )

    official_rsp_launch = os.path.join(
        turtlebot3_gazebo_share,
        "launch",
        "robot_state_publisher.launch.py",
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            official_rsp_launch
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_rgb_optical_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "-1.57079632679",
            "--pitch", "0.0",
            "--yaw", "-1.57079632679",
            "--frame-id", "camera_rgb_frame",
            "--child-frame-id", "camera_rgb_optical_frame",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time.",
            ),

            SetEnvironmentVariable(
                name="TURTLEBOT3_MODEL",
                value="waffle",
            ),

            robot_state_publisher,
            camera_optical_tf,
        ]
    )
