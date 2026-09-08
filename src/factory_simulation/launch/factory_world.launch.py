#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
启动AutoFactory工厂Gazebo世界。

主要功能：
1. 自动查找工厂、办公室、小房屋和TurtleBot3中的有效Gazebo模型；
2. 只收集具有合法model.config和SDF文件的模型；
3. 在~/.cache中生成经过筛选的Gazebo模型库；
4. 避免将整个/opt/ros/humble/share误认为模型目录；
5. 直接启动gzserver和gzclient；
6. 保留ROS 2仿真时间和spawn_entity服务。
"""

from __future__ import annotations

import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def get_optional_package_share(
    package_name: str,
) -> Path | None:
    """
    尝试获取ROS 2功能包的share目录。

    如果功能包不存在，则返回None，不中断Launch启动。

    参数：
        package_name：
            ROS 2功能包名称。
    """

    try:
        return Path(
            get_package_share_directory(package_name)
        )

    except PackageNotFoundError:
        return None


def get_model_sdf_path(
    model_directory: Path,
) -> Path | None:
    """
    从model.config中读取模型使用的SDF文件。

    参数：
        model_directory：
            某个Gazebo模型目录。

    返回：
        合法SDF文件的完整路径；
        如果model.config无效则返回None。
    """

    config_path = model_directory / "model.config"

    if not config_path.is_file():
        return None

    try:
        config_tree = ET.parse(config_path)
        config_root = config_tree.getroot()

    except (ET.ParseError, OSError):
        return None

    sdf_element = config_root.find("sdf")

    if sdf_element is None:
        return None

    if not sdf_element.text:
        return None

    sdf_filename = sdf_element.text.strip()
    sdf_path = model_directory / sdf_filename

    if not sdf_path.is_file():
        return None

    return sdf_path


def find_valid_model_directories(
    search_root: Path,
) -> Iterable[Path]:
    """
    递归搜索指定目录中的标准Gazebo模型。

    一个目录只有同时满足以下条件才会被接受：
    1. 包含model.config；
    2. model.config能够正常解析；
    3. model.config指定的SDF文件真实存在。

    参数：
        search_root：
            准备搜索的根目录。
    """

    if not search_root.is_dir():
        return

    for config_path in search_root.rglob("model.config"):
        model_directory = config_path.parent

        if get_model_sdf_path(model_directory) is None:
            continue

        yield model_directory.resolve()


def clear_generated_library(
    library_root: Path,
) -> None:
    """
    清理上一次Launch生成的模型链接库。

    此目录位于：
    ~/.cache/autofactory_gazebo/model_library

    它仅用于自动生成，因此每次启动时可以重建。
    """

    if library_root.exists():
        shutil.rmtree(library_root)

    library_root.mkdir(
        parents=True,
        exist_ok=True,
    )


def build_filtered_model_library(
    search_roots: list[Path],
    library_root: Path,
) -> tuple[list[str], list[str]]:
    """
    将搜索到的有效模型链接到统一模型库。

    参数：
        search_roots：
            准备搜索的目录列表。

        library_root：
            自动生成的Gazebo模型库目录。

    返回：
        model_names：
            成功加入模型库的目录名称。

        duplicate_messages：
            模型重名记录。
    """

    clear_generated_library(library_root)

    model_names: list[str] = []
    duplicate_messages: list[str] = []
    registered_names: dict[str, Path] = {}

    for search_root in search_roots:
        if not search_root.is_dir():
            continue

        for model_directory in find_valid_model_directories(
            search_root
        ):
            model_name = model_directory.name

            # 如果已经有同名模型，保留优先搜索到的版本
            if model_name in registered_names:
                existing_path = registered_names[model_name]

                if existing_path != model_directory:
                    duplicate_messages.append(
                        f"{model_name}: "
                        f"保留 {existing_path}，"
                        f"忽略 {model_directory}"
                    )

                continue

            link_path = library_root / model_name

            try:
                link_path.symlink_to(
                    model_directory,
                    target_is_directory=True,
                )

            except OSError as error:
                print(
                    f"[AutoFactory] 无法创建模型链接："
                    f"{model_directory}，原因：{error}"
                )
                continue

            registered_names[model_name] = model_directory
            model_names.append(model_name)

    model_names.sort()

    return model_names, duplicate_messages


def generate_launch_description() -> LaunchDescription:
    """创建ROS 2 LaunchDescription对象。"""

    # 当前factory_simulation功能包安装后的share路径
    factory_share = Path(
        get_package_share_directory(
            "factory_simulation"
        )
    )

    # 默认工厂World
    default_world_path = (
        factory_share
        / "worlds"
        / "autofactory.world"
    )

    # 当前项目模型目录
    factory_models_path = (
        factory_share
        / "models"
    )

    # 用户主目录
    home_directory = Path.home()

    # ---------------------------------------------------------
    # 1. 设置需要搜索的模型来源
    # ---------------------------------------------------------

    search_roots: list[Path] = [
        # 当前AutoFactory项目模型
        factory_models_path,

        # 用户通过Gazebo下载或保存的模型
        home_directory / ".gazebo" / "models",

        # 原办公室项目
        home_directory / "gazebo_office",

        # 原小房屋项目
        home_directory / "gazebo_small_house",

        # Gazebo Classic系统模型
        Path("/usr/share/gazebo-11/models"),
    ]

    # ---------------------------------------------------------
    # 2. 加入TurtleBot3 Gazebo模型
    # ---------------------------------------------------------

    turtlebot3_gazebo_share = get_optional_package_share(
        "turtlebot3_gazebo"
    )

    if turtlebot3_gazebo_share is not None:
        search_roots.append(
            turtlebot3_gazebo_share
        )

    # TurtleBot3机械臂仿真包中可能也有有效模型。
    # 这里只递归寻找有效model.config，
    # 不再使用该包错误导出的${prefix}/..路径。
    turtlebot3_manipulation_share = (
        get_optional_package_share(
            "turtlebot3_manipulation_gazebo"
        )
    )

    if turtlebot3_manipulation_share is not None:
        search_roots.append(
            turtlebot3_manipulation_share
        )

    # ---------------------------------------------------------
    # 3. 可选的其他已安装模型功能包
    # ---------------------------------------------------------

    optional_model_packages = [
        "turtlebot3_description",
    ]

    for package_name in optional_model_packages:
        package_share = get_optional_package_share(
            package_name
        )

        if package_share is not None:
            search_roots.append(package_share)

    # ---------------------------------------------------------
    # 4. 创建经过筛选的模型库
    # ---------------------------------------------------------

    generated_library_root = (
        home_directory
        / ".cache"
        / "autofactory_gazebo"
        / "model_library"
    )

    model_names, duplicate_messages = (
        build_filtered_model_library(
            search_roots=search_roots,
            library_root=generated_library_root,
        )
    )

    # Gazebo最终只扫描自动生成的有效模型库
    clean_model_path = str(
        generated_library_root
    )

    # 在Python加载Launch时输出模型统计信息
    print("=" * 80)
    print("[AutoFactory] Gazebo模型库生成完成")
    print(
        f"[AutoFactory] 有效模型数量："
        f"{len(model_names)}"
    )
    print(
        f"[AutoFactory] 模型库路径："
        f"{generated_library_root}"
    )

    turtlebot_models = [
        name
        for name in model_names
        if (
            "turtlebot3" in name.lower()
            or "waffle" in name.lower()
            or "burger" in name.lower()
        )
    ]

    if turtlebot_models:
        print("[AutoFactory] 找到TurtleBot3模型：")

        for model_name in turtlebot_models:
            print(f"  - {model_name}")

    else:
        print(
            "[AutoFactory] 警告："
            "未搜索到带model.config的TurtleBot3模型。"
        )

    if duplicate_messages:
        print(
            f"[AutoFactory] 检测到"
            f"{len(duplicate_messages)}个重名模型，"
            "已按搜索优先级处理。"
        )

    print("=" * 80)

    # ---------------------------------------------------------
    # 5. Launch参数
    # ---------------------------------------------------------

    declare_world_argument = DeclareLaunchArgument(
        "world",
        default_value=str(default_world_path),
        description=(
            "Absolute path to the Gazebo world file."
        ),
    )

    declare_gui_argument = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description=(
            "Whether to start the Gazebo client GUI."
        ),
    )

    # ---------------------------------------------------------
    # 6. 环境变量
    # ---------------------------------------------------------

    set_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=clean_model_path,
    )

    # 不从在线Gazebo模型数据库下载资源
    disable_model_database = SetEnvironmentVariable(
        name="GAZEBO_MODEL_DATABASE_URI",
        value="",
    )

    # VMware和Wayland环境下使用X11兼容后端
    force_x11 = SetEnvironmentVariable(
        name="QT_QPA_PLATFORM",
        value="xcb",
    )

    print_model_path = LogInfo(
        msg=[
            "\n",
            "=" * 72,
            "\nAutoFactory filtered GAZEBO_MODEL_PATH:\n",
            clean_model_path,
            "\nValid model count: ",
            str(len(model_names)),
            "\nTurtleBot3 models: ",
            ", ".join(turtlebot_models)
            if turtlebot_models
            else "not found",
            "\n",
            "=" * 72,
        ]
    )

    # ---------------------------------------------------------
    # 7. 启动Gazebo Server
    # ---------------------------------------------------------

    gzserver_process = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            LaunchConfiguration("world"),
        ],
        output="screen",
        name="autofactory_gzserver",
    )

    # ---------------------------------------------------------
    # 8. 启动Gazebo Client
    # ---------------------------------------------------------

    gzclient_process = ExecuteProcess(
        cmd=[
            "gzclient",
        ],
        output="screen",
        name="autofactory_gzclient",
        condition=IfCondition(
            LaunchConfiguration("gui")
        ),
    )

    # gzserver退出后结束整个Launch
    shutdown_when_server_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=gzserver_process,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="Gazebo server exited."
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            declare_world_argument,
            declare_gui_argument,
            set_model_path,
            disable_model_database,
            force_x11,
            print_model_path,
            gzserver_process,
            gzclient_process,
            shutdown_when_server_exits,
        ]
    )
