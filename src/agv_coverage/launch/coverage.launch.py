#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
启动 agv_coverage 覆盖路径规划节点。

注意：
本 launch 只启动覆盖节点，不启动 Gazebo、Nav2、RViz。
Gazebo、机器人、Nav2、RViz 仍然由 start_autofactory_rpp.sh 启动。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("agv_coverage")

    params_file = os.path.join(
        package_share,
        "config",
        "coverage_params.yaml",
    )

    coverage_node = Node(
        package="agv_coverage",
        executable="coverage_node",
        name="coverage_node",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([coverage_node])
