#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
清理原始工厂Gazebo World。

主要操作：
1. 读取原始SDF World文件；
2. 删除与本项目无关的物体回收模型；
3. 删除原World保存的瞬时state状态；
4. 将世界名称改为autofactory；
5. 输出适合作为ROS 2项目基础环境的新World文件。
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


# 不属于本项目，并且依赖额外插件的模型名称
MODELS_TO_REMOVE = {
    "deletion_wall",
    "drone_collection_zone",
}


def parse_arguments() -> argparse.Namespace:
    """
    解析命令行参数。

    返回：
        argparse.Namespace：
            input_world：原始World文件路径；
            output_world：清理后的World文件路径。
    """

    parser = argparse.ArgumentParser(
        description="Prepare and clean the factory Gazebo world."
    )

    parser.add_argument(
        "input_world",
        type=Path,
        help="Path to the original SDF world file.",
    )

    parser.add_argument(
        "output_world",
        type=Path,
        help="Path to the cleaned output world file.",
    )

    return parser.parse_args()


def load_world(input_path: Path) -> tuple[ET.ElementTree, ET.Element]:
    """
    读取并解析SDF World。

    参数：
        input_path：
            原始World文件路径。

    返回：
        tree：
            XML文档树对象；
        world：
            <world>元素对象。

    异常：
        FileNotFoundError：
            输入文件不存在；
        ValueError：
            文件中没有找到<world>元素；
        ET.ParseError：
            XML/SDF语法错误。
    """

    if not input_path.is_file():
        raise FileNotFoundError(
            f"Input world does not exist: {input_path}"
        )

    tree = ET.parse(input_path)
    root = tree.getroot()

    world = root.find("world")

    if world is None:
        raise ValueError(
            "The input file does not contain a <world> element."
        )

    return tree, world


def remove_unneeded_models(world: ET.Element) -> list[str]:
    """
    删除不需要的顶层模型。

    参数：
        world：
            SDF中的<world>元素。

    返回：
        removed_names：
            实际删除的模型名称列表。
    """

    removed_names: list[str] = []

    # list()生成副本，避免遍历过程中直接修改原列表
    for model in list(world.findall("model")):
        model_name = model.get("name", "")

        if model_name in MODELS_TO_REMOVE:
            world.remove(model)
            removed_names.append(model_name)

    return removed_names


def remove_saved_states(world: ET.Element) -> int:
    """
    删除Gazebo保存的瞬时<state>状态。

    <state>通常包含保存世界时模型的瞬时位置、速度、
    加速度和力，不应作为基础仿真环境的一部分。

    参数：
        world：
            SDF中的<world>元素。

    返回：
        removed_count：
            删除的<state>元素数量。
    """

    removed_count = 0

    for state_element in list(world.findall("state")):
        world.remove(state_element)
        removed_count += 1

    return removed_count


def write_world(
    tree: ET.ElementTree,
    output_path: Path,
) -> None:
    """
    将处理后的SDF写入文件。

    参数：
        tree：
            XML文档树；
        output_path：
            输出文件路径。
    """

    output_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    # Python 3.9以上支持，用于美化XML缩进
    ET.indent(
        tree,
        space="  ",
    )

    tree.write(
        output_path,
        encoding="utf-8",
        xml_declaration=True,
    )


def main() -> int:
    """
    程序入口。

    返回：
        0：处理成功；
        1：处理失败。
    """

    args = parse_arguments()

    try:
        tree, world = load_world(args.input_world)

        original_world_name = world.get("name", "unnamed")

        # 修改正式世界名称
        world.set("name", "autofactory")

        removed_models = remove_unneeded_models(world)
        removed_state_count = remove_saved_states(world)

        write_world(
            tree=tree,
            output_path=args.output_world,
        )

        print("=" * 60)
        print("Factory world preparation completed.")
        print(f"Input world : {args.input_world}")
        print(f"Output world: {args.output_world}")
        print(f"Old name    : {original_world_name}")
        print("New name    : autofactory")
        print(
            "Removed models: "
            + (
                ", ".join(removed_models)
                if removed_models
                else "none"
            )
        )
        print(f"Removed <state> blocks: {removed_state_count}")
        print("=" * 60)

        return 0

    except (
        FileNotFoundError,
        ValueError,
        ET.ParseError,
        OSError,
    ) as error:
        print(
            f"[ERROR] Failed to prepare factory world: {error}",
            file=sys.stderr,
        )

        return 1


if __name__ == "__main__":
    raise SystemExit(main())
