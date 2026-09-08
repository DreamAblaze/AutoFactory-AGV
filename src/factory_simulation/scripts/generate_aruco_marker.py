#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
生成AutoFactory停车位使用的ArUco标记。

字典：DICT_6X6_250
ID：0
输出图像：960 × 960
中心Marker：800 × 800
外侧白边：每边80像素
"""

from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    output_path = Path(
        "/home/my-ubuntu/autofactory_ws/src/"
        "factory_simulation/models/aruco_marker_0/"
        "materials/textures/aruco_0.png"
    )

    output_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_6X6_250
    )

    marker_size_pixels = 800

    # 兼容不同OpenCV版本
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker_image = cv2.aruco.generateImageMarker(
            dictionary,
            0,
            marker_size_pixels,
        )
    else:
        marker_image = np.zeros(
            (marker_size_pixels, marker_size_pixels),
            dtype=np.uint8,
        )

        cv2.aruco.drawMarker(
            dictionary,
            0,
            marker_size_pixels,
            marker_image,
            1,
        )

    # 创建带白色留边的画布
    canvas_size = 960
    margin = 80

    canvas = np.full(
        (canvas_size, canvas_size),
        255,
        dtype=np.uint8,
    )

    canvas[
        margin:margin + marker_size_pixels,
        margin:margin + marker_size_pixels,
    ] = marker_image

    success = cv2.imwrite(
        str(output_path),
        canvas,
    )

    if not success:
        raise RuntimeError(
            f"Failed to write image: {output_path}"
        )

    print("ArUco marker generated successfully.")
    print("Dictionary: DICT_6X6_250")
    print("Marker ID: 0")
    print(f"Output: {output_path}")


if __name__ == "__main__":
    main()
