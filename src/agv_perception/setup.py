from glob import glob
import os

from setuptools import (
    find_packages,
    setup,
)


package_name = "agv_perception"


setup(
    name=package_name,

    version="0.2.0",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [
                "resource/"
                + package_name
            ],
        ),

        (
            "share/" + package_name,
            ["package.xml"],
        ),

        (
            os.path.join(
                "share",
                package_name,
                "config",
            ),
            glob("config/*.yaml"),
        ),

        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob("launch/*.py"),
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="my-ubuntu",

    maintainer_email=(
        "my-ubuntu@example.com"
    ),

    description=(
        "ArUco detection and map-frame "
        "localization for AutoFactory AGV."
    ),

    license="Apache-2.0",

    tests_require=[
        "pytest",
    ],

    entry_points={
        "console_scripts": [
            (
                "aruco_detector = "
                "agv_perception."
                "aruco_detector:main"
            ),

            (
                "aruco_map_transformer = "
                "agv_perception."
                "aruco_map_transformer:main"
            ),
        ],
    },
)
