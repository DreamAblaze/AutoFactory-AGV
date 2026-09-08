from glob import glob
import os

from setuptools import setup

package_name = "agv_coverage"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="my-ubuntu",
    maintainer_email="my-ubuntu@example.com",
    description="Boustrophedon-style grid coverage planner for AutoFactory AGV.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "coverage_node = agv_coverage.coverage_node:main",
        ],
    },
)
