# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is the WATonomous ASD (Autonomous Systems Division) training monorepo — a ROS2-based robot simulation stack using Docker. It uses `watod` (a wrapper around `docker compose`) to manage containerized ROS2 nodes.

## Development Commands

All development happens inside Docker containers. The `watod` CLI wraps `docker compose`.

```bash
# Start containers (uses ACTIVE_MODULES from watod-config.sh)
watod up

# Start a specific service and open a shell into it
watod up <SERVICE_NAME>
watod -t <SERVICE_NAME>

# Inside a container — build all ROS2 packages
colcon build

# Inside a container — source install after building
source install/setup.bash

# Inside a container — run tests (gtest + linting)
colcon test
colcon test-result --verbose

# Inside a container — build without tests
colcon build --cmake-args -DBUILD_TESTING=OFF
```

## Configuration

- `watod-config.sh` — sets `ACTIVE_MODULES` (e.g., `"robot gazebo vis_tools"`) and other compose options.
- `watod-config.local.sh` — gitignored local override; copy from `watod-config.sh` to customize without affecting git.
- `docker-compose.override.yaml` — gitignored local compose overrides.
- Each node has its own `config/params.yaml` loaded at launch via its launch file.

## Architecture

### Stack Modules (`modules/`)

| Module | Description |
|---|---|
| `robot` | Robot perception/planning/control nodes |
| `gazebo` | Ignition Gazebo simulator |
| `vis_tools` | Foxglove visualization bridge |
| `samples` | Reference implementation nodes |

### Robot Node Pipeline (`src/robot/`)

Launched together by `bringup_robot/launch/robot.launch.py`:

```
Gazebo (LaserScan + odometry)
    -> costmap        (LaserScan -> occupancy grid)
    -> map_memory     (accumulates costmaps over time)
    -> planner        (global path planning via A*)
    -> control        (Pure Pursuit -> /cmd_vel)
    + odometry_spoof  (publishes fake odometry for testing)
```

### WATO Core/Node Paradigm

Every ROS2 node is split into two files:
- `*_core.cpp/hpp` — pure algorithm logic, no `rclcpp` calls (unit-testable without spinning ROS2)
- `*_node.cpp/hpp` — ROS2 interface: publishers, subscribers, timers, parameters

The node instantiates the core as a member and delegates to it. This pattern enables gtest unit tests on the core logic without a running ROS2 environment.

### Sample Nodes (`src/samples/`)

Reference implementations (both C++ and Python) showing the Core/Node paradigm, parameters, pub/sub, and testing. Use these when creating new nodes:
- `producer` — publishes data on a timer, handles dynamic parameter updates
- `transformer` — subscribes, buffers, batches, and republishes
- `aggregator` — subscribes to multiple topics, has gtest unit tests in `test/`

### CMakeLists.txt Pattern

Each package follows this structure:
1. Build `*_lib` as a static library from `*_core.cpp`
2. Build `*_node` executable linking against `*_lib`
3. If `BUILD_TESTING=ON`: add gtest via `ament_add_gtest`, link against `*_lib`
4. Install targets and config/launch dirs

### Docker Build Stages

`docker/robot/robot.Dockerfile` uses three stages: `source` (copy + rosdep scan) → `dependencies` (apt install) → `build` (colcon build with Release mode). The built artifacts go to `${WATONOMOUS_INSTALL}`.

Base image: `ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04` (ROS2 Humble).

---

## Node Implementation Details

### Costmap Node (`src/robot/costmap/`)

**Topics:**
- Subscribes: `/lidar` (`sensor_msgs::msg::LaserScan`)
- Publishes: `/costmap` (`nav_msgs::msg::OccupancyGrid`)

**Algorithm:**
1. On each LaserScan message, reset the costmap to all zeros.
2. For each valid range reading, compute Cartesian coordinates:
   - `x = range * cos(angle)`, `y = range * sin(angle)`
   - Convert to grid indices using resolution and origin offset.
3. Mark obstacle cells at cost 100.
4. Inflate obstacles: for each obstacle cell, assign a linearly decreasing cost to surrounding cells within `inflation_radius`:
   - `cost = max_cost * (1 - distance / inflation_radius)`
   - Only overwrite if the new cost is higher than the existing value.
5. Publish the grid with `frame_id = "base_link"`.

**Key parameters (hardcoded in core constructor):**
- `resolution = 0.1` m/cell, grid `200x200` cells, origin at `(-10, -10)` m
- `obstacle_cost = 100`, `inflation_radius = 1.0` m

**Files:** `costmap_core.hpp/cpp`, `costmap_node.hpp/cpp`

---

### Map Memory Node (`src/robot/map_memory/`)

**Topics:**
- Subscribes: `/costmap` (`nav_msgs::msg::OccupancyGrid`), `/odom/filtered` (`nav_msgs::msg::Odometry`)
- Publishes: `/map` (`nav_msgs::msg::OccupancyGrid`)
- Timer: 1 second interval

**Algorithm:**
1. Store latest costmap and robot pose from subscriptions.
2. Track distance traveled since last map update:
   - `distance = sqrt((x_curr - x_last)^2 + (y_curr - y_last)^2)`
   - Set `should_update = true` when distance >= 1.5 m.
3. On timer: if `should_update && costmap_received`, integrate costmap into global map.
4. Integration transforms each costmap cell into the global frame:
   - Robot-local coordinates: `(cx + 0.5) * res + cm_origin`
   - Rotate and translate using robot's `(x, y, theta)`:
     `global_x = robot_x + local_x * cos(theta) - local_y * sin(theta)`
   - Unknown cells (`val < 0`) in the costmap are skipped (retain old global map value).
5. Publish updated global map.

**Key parameters:** global map `2000x200` cells, `0.1` m/cell, origin at `(-100, -100)` m, `distance_threshold = 1.5` m.

**Files:** `map_memory_core.hpp/cpp`, `map_memory_node.hpp/cpp`

---

### Planner Node (`src/robot/planner/`)

**Topics:**
- Subscribes: `/map` (`nav_msgs::msg::OccupancyGrid`), `/goal_point` (`geometry_msgs::msg::PointStamped`), `/odom/filtered` (`nav_msgs::msg::Odometry`)
- Publishes: `/path` (`nav_msgs::msg::Path`)
- Timer: 500 ms interval

**State Machine:**
- `WAITING_FOR_GOAL` — idle, does nothing until a goal arrives.
- `NAVIGATING` — replans on every map update and every timer tick; transitions back when goal is reached (within `goal_tolerance = 0.5` m).

**A* Implementation:**
- 8-connectivity (cardinal cost 1.0, diagonal cost √2).
- Heuristic: Euclidean distance to goal.
- Cells with cost >= `occupied_threshold` (90) are treated as impassable; unknown cells (`-1`) are treated as passable (for exploration).
- Supporting structs in `planner_core.hpp`: `CellIndex`, `CellIndexHash`, `AStarNode`, `CompareF`.
- Path is reconstructed by tracing `came_from` map and converting grid indices back to world coordinates.

**Files:** `planner_core.hpp/cpp`, `planner_node.hpp/cpp`

---

### Control Node (`src/robot/control/`)

**Topics:**
- Subscribes: `/path` (`nav_msgs::msg::Path`), `/odom/filtered` (`nav_msgs::msg::Odometry`)
- Publishes: `/cmd_vel` (`geometry_msgs::msg::Twist`)
- Timer: 100 ms interval (10 Hz)

**Algorithm — Pure Pursuit:**
1. Find the closest waypoint on the path to the robot.
2. Starting from that waypoint, find the first one at or beyond `lookahead_distance`.
3. If none exists, use the final waypoint.
4. Transform the lookahead point into the robot frame:
   - `local_x = dx*cos(theta) + dy*sin(theta)`
   - `local_y = -dx*sin(theta) + dy*cos(theta)`
5. Compute curvature: `kappa = 2 * sin(alpha) / ld` where `alpha = atan2(local_y, local_x)`.
6. `angular_vel = linear_speed * kappa`, clamped to `[-max_angular_speed, max_angular_speed]`.
7. Stop (zero velocity) if goal is within `goal_tolerance`.

**Key parameters:** `lookahead_distance = 1.0` m, `linear_speed = 0.5` m/s, `max_angular_speed = 1.0` rad/s, `goal_tolerance = 0.3` m.

**Files:** `control_core.hpp/cpp`, `control_node.hpp/cpp`

---

## Topic Map

```
/lidar (LaserScan)
    -> [costmap_node] -> /costmap (OccupancyGrid)
                              -> [map_memory_node] -> /map (OccupancyGrid)
/odom/filtered (Odometry)                                -> [planner_node] -> /path (Path)
    -> [map_memory_node]                                                         -> [control_node] -> /cmd_vel (Twist)
    -> [planner_node]
    -> [control_node]
/goal_point (PointStamped)
    -> [planner_node]
```
