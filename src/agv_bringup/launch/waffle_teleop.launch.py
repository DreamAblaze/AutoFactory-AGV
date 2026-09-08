#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
生成AutoFactory Waffle机器人，然后打开独立键盘控制终端。

使用前提：
1. 工厂Gazebo世界已经启动；
2. /spawn_entity和/delete_entity服务已经存在；
3. agv_bringup功能包已经编译。
"""

from __future__ import annotations

import shutil

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """创建机器人生成与键盘控制Launch描述。"""

    # GNOME终端用于承载需要实时接收键盘输入的teleop程序
    if shutil.which("gnome-terminal") is None:
        raise RuntimeError(
            "没有找到gnome-terminal。请执行："
            "sudo apt install gnome-terminal"
        )

    # ---------------------------------------------------------
    # 1. 可在命令行中覆盖的参数
    # ---------------------------------------------------------

    declared_arguments = [
        DeclareLaunchArgument(
            "entity_name",
            default_value="autofactory_agv",
            description="机器人在Gazebo中的实体名称。",
        ),
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="",
            description="机器人ROS 2命名空间。",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="8.738000",
            description="机器人初始X坐标。",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="7.489490",
            description="机器人初始Y坐标。",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0.050000",
            description="机器人初始Z坐标。",
        ),
        DeclareLaunchArgument(
            "roll",
            default_value="0.0",
            description="机器人初始Roll角。",
        ),
        DeclareLaunchArgument(
            "pitch",
            default_value="0.0",
            description="机器人初始Pitch角。",
        ),
        DeclareLaunchArgument(
            "yaw",
            default_value="3.141593",
            description="机器人初始Yaw角，默认旋转180度。",
        ),
        DeclareLaunchArgument(
            "delete_existing",
            default_value="true",
            description="生成前是否删除同名旧机器人。",
        ),
        DeclareLaunchArgument(
            "service_timeout_sec",
            default_value="60.0",
            description="等待Gazebo服务的最长时间。",
        ),
    ]

    # ---------------------------------------------------------
    # 2. 机器人生成节点
    # ---------------------------------------------------------

    spawn_node = Node(
        package="agv_bringup",
        executable="spawn_waffle",
        name="spawn_autofactory_waffle",
        output="screen",
        parameters=[
            {
                "entity_name": LaunchConfiguration(
                    "entity_name"
                ),
                "robot_namespace": LaunchConfiguration(
                    "robot_namespace"
                ),

                # ParameterValue显式指定类型，
                # 避免Launch把数字参数当作字符串传入
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
                    LaunchConfiguration(
                        "delete_existing"
                    ),
                    value_type=bool,
                ),
                "service_timeout_sec": ParameterValue(
                    LaunchConfiguration(
                        "service_timeout_sec"
                    ),
                    value_type=float,
                ),
            }
        ],
    )

    # ---------------------------------------------------------
    # 3. 键盘控制进程
    # ---------------------------------------------------------

    # 新终端会继承当前Launch进程中的ROS环境。
    # teleop_keyboard必须处于交互式终端中才能接收按键。
    teleop_command = (
        "export TURTLEBOT3_MODEL=waffle; "
        "exec ros2 run "
        "turtlebot3_teleop teleop_keyboard"
    )

    teleop_terminal = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--title=AutoFactory Waffle Keyboard",
            "--",
            "bash",
            "-lc",
            teleop_command,
        ],
        output="screen",
    )

    # ---------------------------------------------------------
    # 4. 机器人生成节点退出后的处理
    # ---------------------------------------------------------

    def handle_spawn_exit(event, context):
        """
        只有机器人生成节点正常退出时，才启动键盘控制。
        """

        del context

        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        "Waffle机器人生成成功，"
                        "正在打开键盘控制终端。"
                    )
                ),
                teleop_terminal,
            ]

        # ROS 2 Humble没有launch.actions.LogError，
        # 因此这里使用LogInfo并在文本中明确标记错误。
        return [
            LogInfo(
                msg=(
                    "[ERROR] Waffle机器人生成失败，"
                    "不会启动键盘控制节点。"
                )
            )
        ]

    start_teleop_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_node,
            on_exit=handle_spawn_exit,
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            SetEnvironmentVariable(
                name="TURTLEBOT3_MODEL",
                value="waffle",
            ),
            spawn_node,
            start_teleop_after_spawn,
        ]
    )
