#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Spawn the AutoFactory TurtleBot3 Waffle in an already running Gazebo world."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node


def quaternion_from_rpy(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    """Convert roll, pitch and yaw to quaternion x, y, z and w."""

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return qx, qy, qz, qw


class WaffleSpawner(Node):
    """Create one Waffle entity through Gazebo ROS services."""

    def __init__(self) -> None:
        super().__init__("autofactory_waffle_spawner")

        self.declare_parameter("entity_name", "autofactory_agv")
        self.declare_parameter("robot_namespace", "")
        self.declare_parameter("x", 8.738000)
        self.declare_parameter("y", 7.489490)
        self.declare_parameter("z", 0.050000)
        self.declare_parameter("roll", 0.0)      # 滚转角
        self.declare_parameter("pitch", 0.0)     # 倾斜角
        self.declare_parameter("yaw", 3.141593)  # 偏航角
        self.declare_parameter("delete_existing", True)
        self.declare_parameter("service_timeout_sec", 60.0)

        self.entity_name = str(
            self.get_parameter("entity_name").value
        )
        self.robot_namespace = str(
            self.get_parameter("robot_namespace").value
        )
        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.z = float(self.get_parameter("z").value)
        self.roll = float(self.get_parameter("roll").value)
        self.pitch = float(self.get_parameter("pitch").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.delete_existing = bool(
            self.get_parameter("delete_existing").value
        )
        self.timeout = float(
            self.get_parameter("service_timeout_sec").value
        )

        self.spawn_client = self.create_client(
            SpawnEntity,
            "/spawn_entity",
        )
        self.delete_client = self.create_client(
            DeleteEntity,
            "/delete_entity",
        )

    def load_waffle_sdf(self) -> str:
        """Load model.sdf and convert shared model URIs to absolute file URIs."""

        turtlebot3_share = Path(
            get_package_share_directory("turtlebot3_gazebo")
        )

        waffle_sdf_path = Path(
            "/home/my-ubuntu/autofactory_ws/src/agv_bringup/models/model.sdf"
        )

        common_resource_path = (
            turtlebot3_share
            / "models"
            / "turtlebot3_common"
        )

        if not waffle_sdf_path.is_file():
            raise FileNotFoundError(
                f"Waffle SDF does not exist: {waffle_sdf_path}"
            )

        if not common_resource_path.is_dir():
            raise FileNotFoundError(
                "TurtleBot3 common resource directory does not exist: "
                f"{common_resource_path}"
            )

        sdf_text = waffle_sdf_path.read_text(encoding="utf-8")

        # Avoid the repeated "invisible Waffle" problem by replacing
        # model://turtlebot3_common with a package-resolved absolute file URI.
        common_file_uri = common_resource_path.resolve().as_uri()
        sdf_text = sdf_text.replace(
            "model://turtlebot3_common",
            common_file_uri,
        )

        self.get_logger().info(
            f"Using Waffle model: {waffle_sdf_path}"
        )
        self.get_logger().info(
            f"Resolved common meshes to: {common_file_uri}"
        )

        return sdf_text

    def wait_for_service(self, client, service_name: str) -> bool:
        """Wait for one Gazebo service."""

        self.get_logger().info(
            f"Waiting for {service_name}, timeout={self.timeout:.1f}s"
        )

        ready = client.wait_for_service(
            timeout_sec=self.timeout
        )

        if not ready:
            self.get_logger().error(
                f"{service_name} is unavailable. "
                "Start factory_world.launch.py first."
            )

        return ready

    def delete_old_entity(self) -> None:
        """Attempt to delete an old entity with the same name."""

        if not self.wait_for_service(
            self.delete_client,
            "/delete_entity",
        ):
            return

        request = DeleteEntity.Request()
        request.name = self.entity_name

        future = self.delete_client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.timeout,
        )

        if not future.done() or future.result() is None:
            self.get_logger().warning(
                "DeleteEntity did not return a result."
            )
            return

        response = future.result()

        if response.success:
            self.get_logger().info(
                f"Deleted old entity: {self.entity_name}"
            )
        else:
            # It is normal when this entity has not been spawned yet.
            self.get_logger().info(
                "No old entity was deleted: "
                f"{response.status_message}"
            )

    def spawn(self) -> bool:
        """Send the SpawnEntity request."""

        if not self.wait_for_service(
            self.spawn_client,
            "/spawn_entity",
        ):
            return False

        request = SpawnEntity.Request()
        request.name = self.entity_name
        request.xml = self.load_waffle_sdf()
        request.robot_namespace = self.robot_namespace
        request.reference_frame = "world"

        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z

        qx, qy, qz, qw = quaternion_from_rpy(
            self.roll,
            self.pitch,
            self.yaw,
        )
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        request.initial_pose = pose

        self.get_logger().info(
            "Spawning AutoFactory Waffle: "
            f"name={self.entity_name}, "
            f"x={self.x:.6f}, y={self.y:.6f}, z={self.z:.6f}, "
            f"yaw={self.yaw:.6f}"
        )

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.timeout,
        )

        if not future.done() or future.result() is None:
            self.get_logger().error(
                "SpawnEntity did not return a result."
            )
            return False

        response = future.result()

        if response.success:
            self.get_logger().info(
                f"Robot spawned successfully: {response.status_message}"
            )
            return True

        self.get_logger().error(
            f"Robot spawn failed: {response.status_message}"
        )
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaffleSpawner()
    exit_code = 1

    try:
        if node.delete_existing:
            node.delete_old_entity()

        exit_code = 0 if node.spawn() else 1

    except Exception as error:
        node.get_logger().error(str(error))
        exit_code = 1

    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
