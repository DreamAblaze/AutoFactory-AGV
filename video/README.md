# AutoFactory AGV Demo Videos

This directory provides information about the demonstration videos for the **AutoFactory AGV Full-Coverage Inspection and Autonomous Visual-Guided Parking System**.

To keep the Git repository lightweight, the original MP4 files are **not stored directly in the source repository**.  
The full-resolution videos are distributed through the GitHub **Releases** page.

---

## Available Demonstrations

| Video | Demonstration |
|---|---|
| `Nav2.mp4` | Nav2 autonomous navigation |
| `agv_coverage.mp4` | Full-coverage inspection |
| `Vision-Guided_Autonomous_Parking.mp4` | Vision-guided autonomous parking |

---

## 1. Nav2.mp4

### Nav2 Autonomous Navigation Demo

`Nav2.mp4` demonstrates the basic autonomous navigation capability of AutoFactory AGV in the Gazebo factory environment.

Main contents:

- Gazebo factory environment startup
- TurtleBot3 Waffle spawning
- Robot TF initialization
- AMCL localization
- Nav2 startup
- Global path planning
- Regulated Pure Pursuit Controller
- Velocity smoothing
- RViz Goal Pose interaction
- Autonomous path following
- Goal arrival

Corresponding startup command:

```bash
cd /home/my-ubuntu/autofactory_ws
./start_autofactory.sh nav2
```

Main pipeline:

```text
Gazebo Factory Environment
        ↓
TurtleBot3 Waffle
        ↓
Robot TF
        ↓
AMCL Localization
        ↓
Nav2
        ↓
Global Path Planning
        ↓
Regulated Pure Pursuit Controller
        ↓
Velocity Smoother
        ↓
Autonomous Navigation
```

Main RViz configuration:

```text
src/agv_bringup/rviz/autofactory_rpp.rviz
```

---

## 2. agv_coverage.mp4

### Full-Coverage Inspection Demo

`agv_coverage.mp4` demonstrates the full-coverage inspection capability of AutoFactory AGV.

The coverage subsystem generates a boustrophedon-style inspection path from the occupancy grid map and executes the path through Nav2.

Main contents:

- Occupancy-grid map loading
- Free-space extraction
- Obstacle inflation
- Coverage sweep direction estimation
- Boustrophedon sweep-line generation
- Coverage path generation
- Coverage waypoint extraction
- Multi-batch waypoint execution
- Nav2 `NavigateThroughPoses`
- Coverage path visualization
- Coverage waypoint visualization
- Autonomous full-area traversal
- Coverage progress reporting
- Coverage mission completion

Corresponding startup command:

```bash
cd /home/my-ubuntu/autofactory_ws
./start_autofactory.sh coverage
```

Main pipeline:

```text
Gazebo Factory Environment
        ↓
TurtleBot3 Waffle
        ↓
TF + AMCL + Nav2
        ↓
Occupancy Grid Map
        ↓
Free-Space Extraction
        ↓
Boustrophedon Coverage Planning
        ↓
Coverage Path
        ↓
Coverage Waypoints
        ↓
Multi-Batch NavigateThroughPoses
        ↓
Autonomous Full-Coverage Inspection
        ↓
Coverage Complete
```

Main coverage topics:

```text
/coverage_path
/coverage_waypoints
/coverage_progress
/coverage_status
```

Main RViz configuration:

```text
src/agv_bringup/rviz/autofactory_coverage.rviz
```

Successful full-coverage terminal message:

```text
本次覆盖执行完成。
```

---

## 3. Vision-Guided_Autonomous_Parking.mp4

### Vision-Guided Autonomous Parking Demo

`Vision-Guided_Autonomous_Parking.mp4` demonstrates the complete autonomous visual-guided parking process.

The robot first navigates to a parking staging position, aligns the onboard camera toward the visual marker, recognizes and locks ArUco ID 0, transforms the detected marker pose into the ROS `map` frame, generates a Hybrid A* parking path, and finally performs autonomous parking.

Main contents:

- Parking staging path planning
- Autonomous navigation to the staging position
- Parking staging position verification
- Final camera heading alignment
- ArUco marker recognition
- ArUco ID 0 detection
- Stable-frame marker locking
- Marker pose estimation
- Marker pose transformation into the `map` frame
- Hybrid A* parking planning
- Forward and reverse motion planning
- Collision checking
- Parking path visualization
- Parking tracker initialization
- Nav2 velocity ownership handoff
- Autonomous parking path tracking
- Final position correction
- Final heading correction
- Parking mission completion

Corresponding startup command:

```bash
cd /home/my-ubuntu/autofactory_ws
./start_autofactory.sh parking
```

Main pipeline:

```text
Nav2 Ready
        ↓
Parking Staging Planning
        ↓
FollowPath
        ↓
Parking Staging Reached
        ↓
Final Camera Heading Alignment
        ↓
ArUco ID 0 Recognition
        ↓
Stable Marker Lock
        ↓
Map-Frame Marker Localization
        ↓
Hybrid A* Planning
        ↓
Parking Path Generation
        ↓
Parking Tracker
        ↓
Autonomous Parking
        ↓
Mission Complete
```

Validated parking staging target:

```text
x   = -6.0
y   = -4.5
yaw = 3.1 rad
```

Validated final parking target in the ROS `map` frame:

```text
x   = -8.357680
y   = -4.166548
yaw = 1.539811
```

Parking tracker version:

```text
V3.2_SAFE_GEOMETRY
```

Main parking visualization topics:

```text
/parking_staging_path
/parking_staging_path_end
/aruco/marker_pose_map
/parking/hybrid_path
```

Main RViz configuration:

```text
src/agv_bringup/rviz/autofactory_rpp.rviz
```

During ArUco recognition, the system also displays:

```text
/aruco/debug_image
```

through `rqt_image_view`.

---

## Complete Autonomous Mission

The complete AutoFactory AGV mission combines the coverage and parking subsystems.

Startup command:

```bash
cd /home/my-ubuntu/autofactory_ws
./start_autofactory.sh full
```

Complete mission pipeline:

```text
Gazebo Factory Environment
        ↓
TurtleBot3 Waffle Spawn
        ↓
Robot TF
        ↓
AMCL + Nav2
        ↓
Full-Coverage Planning
        ↓
Full-Coverage Inspection
        ↓
Coverage Complete
        ↓
Close Coverage RViz
        ↓
Open RPP / Parking RViz
        ↓
Coverage-to-Parking Nav2 Handoff
        ↓
Parking Staging
        ↓
Final Camera Heading Alignment
        ↓
ArUco ID 0 Recognition
        ↓
Stable Marker Lock
        ↓
Map-Frame Localization
        ↓
Hybrid A* Parking Planning
        ↓
Parking Path Tracking
        ↓
Autonomous Parking
        ↓
Mission Complete
```

RViz switching workflow:

```text
Full-Coverage Stage
        ↓
autofactory_coverage.rviz
        ↓
Coverage Complete
        ↓
Coverage RViz Automatically Closed
        ↓
autofactory_rpp.rviz
        ↓
Parking Staging
        ↓
ArUco Recognition
        ↓
Hybrid A* Path
        ↓
Autonomous Parking
```

---

## Download Demonstration Videos

Repository:

```text
https://github.com/DreamAblaze/AutoFactory-AGV
```

Releases page:

```text
https://github.com/DreamAblaze/AutoFactory-AGV/releases
```

Recommended release:

```text
v1.0.0
```

Expected release assets:

```text
Nav2.mp4
agv_coverage.mp4
Vision-Guided_Autonomous_Parking.mp4
```

---

## Why the Videos Are Not Stored in the Git Repository

The demonstration videos are binary files and are not required for compiling or running AutoFactory AGV.

Keeping full-resolution MP4 files directly in Git would unnecessarily increase:

- repository size;
- Git history size;
- clone time;
- download traffic.

Therefore, the source repository contains only:

```text
video/README.md
```

while the original videos are distributed as GitHub Release assets.

---

## Video File Mapping

| File | Module | Startup Mode |
|---|---|---|
| `Nav2.mp4` | Nav2 autonomous navigation | `./start_autofactory.sh nav2` |
| `agv_coverage.mp4` | Full-coverage inspection | `./start_autofactory.sh coverage` |
| `Vision-Guided_Autonomous_Parking.mp4` | Visual-guided autonomous parking | `./start_autofactory.sh parking` |

The complete integrated mission is executed with:

```bash
./start_autofactory.sh full
```

---

## Related Documentation

Main project documentation:

```text
README.md
```

Detailed Chinese project manual:

```text
docs/AutoFactory_AGV_项目使用说明_v1.0.docx
```

---

## Project Information

Project:

```text
AutoFactory AGV
```

Full project name:

```text
AutoFactory AGV 全覆盖巡检与自主识别泊车系统
```

GitHub:

```text
https://github.com/DreamAblaze/AutoFactory-AGV
```

Maintainer:

```text
DreamAblaze
```

Current validated release:

```text
v1.0.0
```

---

## License

The AutoFactory AGV source code is intended to be released under the MIT License.

Please refer to:

```text
LICENSE
```

Third-party Gazebo models, TurtleBot3 assets, and other external resources remain subject to their respective original licenses.

---

## Summary

The three demonstration videos show the three major capabilities of AutoFactory AGV:

```text
Nav2 Autonomous Navigation
        +
Full-Coverage Inspection
        +
Vision-Guided Autonomous Parking
```

Together they form the complete autonomous factory AGV workflow:

```text
Inspection
        ↓
Navigation
        ↓
Visual Recognition
        ↓
Parking Planning
        ↓
Autonomous Parking
```
