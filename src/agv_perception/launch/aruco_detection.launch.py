#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
)

from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import Node

from launch_ros.parameter_descriptions import (
    ParameterValue,
)


def generate_launch_description():

    use_sim_time = LaunchConfiguration(
        "use_sim_time"
    )

    package_share = (
        get_package_share_directory(
            "agv_perception"
        )
    )

    params_file = os.path.join(
        package_share,
        "config",
        "aruco_params.yaml",
    )

    aruco_node = Node(
        package="agv_perception",
        executable="aruco_detector",
        name="aruco_detector",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time":
                    ParameterValue(
                        use_sim_time,
                        value_type=bool,
                    )
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
            ),

            aruco_node,
        ]
    )
