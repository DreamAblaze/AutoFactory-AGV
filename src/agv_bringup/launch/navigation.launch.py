#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    agv_bringup_share = get_package_share_directory("agv_bringup")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_map = os.path.join(
        agv_bringup_share,
        "maps",
        "autofactory_navigation.yaml",
    )

    default_params = os.path.join(
        agv_bringup_share,
        "params",
        "autofactory_nav2.yaml",
    )

    nav2_launch = os.path.join(
        nav2_bringup_share,
        "launch",
        "bringup_launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "params_file": LaunchConfiguration("params_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": LaunchConfiguration("autostart"),
                }.items(),
            ),
        ]
    )
