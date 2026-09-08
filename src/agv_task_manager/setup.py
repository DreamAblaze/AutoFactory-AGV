from glob import glob
import os

from setuptools import find_packages, setup


package_name = "agv_task_manager"


setup(
    name=package_name,
    version="0.1.0",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
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
        "AutoFactory AGV task management "
        "and parking staging navigation."
    ),

    license="Apache-2.0",

    tests_require=[
        "pytest",
    ],

    entry_points={
        "console_scripts": [
            (
                "parking_staging_navigator = "
                "agv_task_manager."
                "parking_staging_navigator:main"
            ),
        ],
    },
)
