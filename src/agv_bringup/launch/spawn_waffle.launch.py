#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Spawn only the AutoFactory Waffle. Gazebo must already be running."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    declared_arguments = [
        DeclareLaunchArgument(
            "entity_name",
            default_value="autofactory_agv",
        ),
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="8.738000",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="7.489490",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0.050000",
        ),
        DeclareLaunchArgument(
            "roll",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "pitch",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "yaw",
            default_value="3.141593",
        ),
        DeclareLaunchArgument(
            "delete_existing",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "service_timeout_sec",
            default_value="60.0",
        ),
    ]

    spawn_node = Node(
        package="agv_bringup",
        executable="spawn_waffle",
        name="spawn_autofactory_waffle",
        output="screen",
        parameters=[
            {
                "entity_name": LaunchConfiguration("entity_name"),
                "robot_namespace": LaunchConfiguration("robot_namespace"),
                "x": ParameterValue(
                    LaunchConfiguration("x"),
                    value_type=float,
                ),
                "y": ParameterValue(
                    LaunchConfiguration("y"),
                    value_type=float,
                ),
                "z": ParameterValue(
                    LaunchConfiguration("z"),
                    value_type=float,
                ),
                "roll": ParameterValue(
                    LaunchConfiguration("roll"),
                    value_type=float,
                ),
                "pitch": ParameterValue(
                    LaunchConfiguration("pitch"),
                    value_type=float,
                ),
                "yaw": ParameterValue(
                    LaunchConfiguration("yaw"),
                    value_type=float,
                ),
                "delete_existing": ParameterValue(
                    LaunchConfiguration("delete_existing"),
                    value_type=bool,
                ),
                "service_timeout_sec": ParameterValue(
                    LaunchConfiguration("service_timeout_sec"),
                    value_type=float,
                ),
            }
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            SetEnvironmentVariable(
                "TURTLEBOT3_MODEL",
                "waffle",
            ),
            spawn_node,
        ]
    )
