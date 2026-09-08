from glob import glob
import os

from setuptools import find_packages, setup


package_name = "agv_bringup"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            os.path.join("share", package_name),
            ["package.xml", "README.md"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "params"),
            glob("params/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "maps"),
            glob("maps/*.yaml") + glob("maps/*.pgm"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "maps",
                "posegraph",
            ),
            glob("maps/posegraph/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AutoFactory Project",
    maintainer_email="wenjing_qin222@163.com",
    description=(
        "Bringup, SLAM, maps, and navigation launch files "
        "for the AutoFactory TurtleBot3 Waffle project."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "spawn_waffle = agv_bringup.spawn_waffle:main",
        ],
    },
)
