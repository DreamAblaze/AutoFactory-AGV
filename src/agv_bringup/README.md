# AutoFactory AGV Bringup

This package is intentionally independent of the factory-world launch package.

## Default initial pose

- x: 8.738000
- y: 7.489490
- z: 0.050000
- yaw: 3.141593

## Commands

Start the factory world first.

Spawn only:

```bash
ros2 launch agv_bringup spawn_waffle.launch.py
```

Spawn and open keyboard teleoperation:

```bash
ros2 launch agv_bringup waffle_teleop.launch.py
```

Temporary pose override:

```bash
ros2 launch agv_bringup waffle_teleop.launch.py \
  x:=8.5 y:=7.2 yaw:=3.141593
```

The spawner always selects TurtleBot3 Waffle `model.sdf` and rewrites
`turtlebot3_common` mesh URIs to absolute file URIs before submitting the
Gazebo `SpawnEntity` request.
