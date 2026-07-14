# Hospital Delivery Robot — ROS 2 (Humble) + Ignition Gazebo

A differential-drive hospital delivery robot built with ROS 2 Humble and Ignition Gazebo (Fortress). The robot navigates autonomously through a simulated hospital environment, performing SLAM-based mapping and Nav2-powered point-to-point delivery. The primary delivery mission is: **spawn → supply table → reception desk** — simulating a real hospital supply run.

---

## Demo

> Robot executing delivery mission: Spawn → Supply Table → Reception Desk

---

## Table of Contents

- [Features](#features)
- [Robot Design](#robot-design)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Running SLAM Mapping](#running-slam-mapping)
- [Running Navigation](#running-navigation)
- [Delivery Mission](#delivery-mission)
- [Sending Navigation Goals](#sending-navigation-goals)
- [Key Locations](#key-locations)
- [Architecture](#architecture)
- [Configuration](#configuration)
- [Known Issues & Fixes](#known-issues--fixes)
- [Troubleshooting](#troubleshooting)

---

## Features

- Custom differential-drive robot with 2D LiDAR (12m range, 360°)
- L-shaped hospital world with patient rooms, nurses station, reception, and charging bay
- SLAM mapping using `slam_toolbox` (async mapping mode)
- Autonomous navigation using Nav2 with:
  - A\* global planner (`nav2_navfn_planner`)
  - DWB local planner with obstacle avoidance
  - AMCL localization on a pre-built map
  - Custom recovery behaviors (spin, backup, drive-on-heading, wait)
- EKF odometry fusion via `robot_localization`
- ROS-Ignition bridge for sensor and actuator communication
- Multi-goal waypoint delivery via `NavigateThroughPoses`
- Primary delivery route: **Spawn → Supply Table → Reception Desk**

---

## Robot Design

### Why a Differential Drive?

A differential drive was chosen as the locomotion system for three reasons. First, it is mechanically simple — only two independently driven wheels with two passive casters, which means fewer failure points and easier simulation. Second, it is well-suited to corridor navigation: the robot only needs to move forward, backward, and rotate in place, and a diff drive covers all of these without requiring complex kinematics. Third, ROS 2 and Ignition Gazebo have first-class support for diff drive through the `libignition-gazebo-diff-drive-system` plugin, making odometry and `cmd_vel` integration straightforward.

### Chassis and Dimensions

The chassis dimensions were designed to fit through standard hospital doorways while carrying a payload box. Key dimensions and masses are taken directly from SolidWorks assembly data:

| Component | Dimension | Real Mass (SolidWorks) |
|---|---|---|
| Chassis | 0.900 × 0.870 × 0.336 m | 11.17 kg |
| Payload box | 0.600 × 0.450 × 0.240 m | 0.25 kg |
| LiDAR sensor | cylinder r=0.060 l=0.180 m | 0.04 kg |
| Each drive wheel | radius 0.150 m, width 0.120 m | 0.35 kg |
| Each caster assembly | sphere r=0.120 m | 0.18 kg |
| **Total robot** | | **12.52 kg** |
| Wheel separation (axle width) | 0.720 m | — |
| Caster wheel radius | 0.120 m | — |

The total robot mass of **12.52 kg** matches the SolidWorks full assembly mass. This is realistic for a small hospital delivery robot — comparable in scale to lightweight delivery platforms used in real hospital environments.

### Caster Wheel Design

Two steerable caster assemblies (front-left, front-right) were used instead of fixed rear casters. Each caster has:
- A swivel joint (continuous, rotates around Z) — allows the fork to self-align with the direction of travel
- A wheel joint (continuous, rotates around Y) — the rolling contact

Both joints use near-zero friction (`mu1=0.0`, `mu2=0.0`) so they self-align passively without requiring any controller. This prevents the robot from dragging its rear during tight turns, which would corrupt odometry readings.

### Payload Box

A fixed payload box (`0.600 × 0.450 × 0.240 m`, mass 0.25 kg) sits on top of the chassis. It is attached via a fixed joint (no movement relative to chassis). Its purpose in simulation is purely visual — to represent the delivery compartment. In a real implementation this would be replaced with a lockable compartment with a door mechanism. It was kept as a static visual to avoid unnecessary joint complexity that would interfere with `joint_state_publisher`.

### LiDAR Sensor Placement

A GPU LiDAR sensor (`gpu_lidar` plugin, mass 0.04 kg) was placed on a dedicated mount (`lidar_mount`) elevated at `z = 0.495m` above the base. This height was chosen to:

1. Clear the payload box (top of payload box is at ~0.660m, lidar is at 0.495m — deliberately below the payload to avoid self-occlusion from behind)
2. Sit above the chassis walls so rays are not blocked by the chassis geometry
3. Keep it low enough to detect ground-level obstacles (chairs, boxes, trolleys) rather than only seeing walls

The sensor has **no collision geometry** on `lidar_link` — a deliberate design choice. Sensor links with collision boxes cause raycasting to register hits at distance zero (the ray immediately intersects its own link). Removing the collision block was the fix that made `/scan` produce real data.

### Sensor Specifications

| Parameter | Value | Reason |
|---|---|---|
| Scan type | 2D (single horizontal plane) | Sufficient for corridor navigation — 3D adds cost and complexity with no benefit for flat-floor environments |
| Angular range | 360° (−π to +π) | Full surround awareness — detects obstacles in all directions including behind |
| Samples | 360 | One ray per degree — good resolution at low computational cost |
| Range | 0.35m – 12.0m | Min range clears robot body; max range covers the full hospital corridor width |
| Update rate | 10 Hz | Sufficient for Nav2 costmap updates; higher rates stress the simulation |
| Noise | Gaussian (σ=0.01m) | Realistic sensor noise for AMCL particle filter tuning |

### Why 2D LiDAR Over Depth Camera?

A depth camera (RGB-D) was considered but rejected for this use case because:
- The hospital environment is a flat floor — depth information in the vertical axis adds no navigational value
- 2D LiDAR produces a clean `sensor_msgs/LaserScan` message that AMCL, slam_toolbox, and Nav2 costmaps all consume natively without any conversion
- GPU LiDAR in Ignition Gazebo is significantly faster to simulate than a point cloud camera at equivalent range
- AMCL's particle filter is specifically designed for 2D laser scans — using it with a depth camera would require an extra projection step

### Hospital World Design

The world is an L-shaped layout representing a small hospital wing. The primary delivery route (Spawn → Supply Table → Reception) is highlighted:

```
┌──────────────┬──────────────┐
│  Patient     │  Patient     │
│  Room 1      │  Room 2      │
│              │              │
├──────────────┴──────────────┤
│                              │
│   Nurses Station  [Desk]     │
│                              │
├──────┐                       │
│Recep-│ ← GOAL 2              │
│tion  │                       │
│      │                       │
│Supply│ ← GOAL 1              │
│Table │                       │
└──────┴───────────────────────┘
    ↑
  SPAWN (0, -2.9)
         │
    Charging Bay / Storage
```

The delivery mission travels: **Spawn (bottom corridor) → Supply Table (reception area, north-west) → Reception Desk (adjacent to supply table)**. This route covers the maximum distance in the navigable corridor and passes through the main junction, making it a good test of long-range path planning and multi-goal navigation.

Every wall and obstacle was given explicit `<collision>` geometry so the LiDAR can detect them correctly. All models are marked `<static>true</static>` so they don't move under physics. The three human actors (`roamer_1`, `stationary_receptionist`, `erratic_patient`) were added for visual realism but their collision with nav2 costmaps was found to cause path planning failures — they are commented out for navigation sessions.

### Why Ignition Gazebo (Fortress) Over Classic Gazebo?

Ignition Fortress was chosen because:
- It is the officially supported simulator for ROS 2 Humble
- The `gpu_lidar` sensor plugin produces significantly more realistic raycast data than Classic Gazebo's ray sensor
- The `ros_gz_bridge` provides clean topic bridging between Ignition and ROS 2
- Classic Gazebo (Gazebo 11) is in maintenance mode and does not receive new sensor or physics updates

The tradeoff is that the `ros_gz_bridge` introduces an extra configuration layer (topic mapping, QoS settings) that Classic Gazebo with `gazebo_ros_pkgs` handles more transparently. Several hours of this project were spent debugging bridge QoS mismatches and `cmd_vel` topic remapping.

### Software Design Decisions

**EKF for odometry:** `robot_localization` EKF was added to fuse wheel odometry from the diff drive plugin. Raw odometry from Ignition can have discontinuities during physics steps — the EKF smooths these and publishes a consistent `odom → base_link` transform. Without EKF, the TF tree was empty on startup.

**AMCL over slam_toolbox localization:** Once a map is saved, AMCL was chosen for localization over slam_toolbox's localization mode because AMCL has lower CPU overhead (particle filter vs. scan matching graph), and its `set_initial_pose` parameter allows automatic initialization at the spawn point without any manual 2D Pose Estimate step.

**DWB over RegulatedPurePursuit:** The DWB (Dynamic Window Approach based) local planner was ultimately chosen over `nav2_regulated_pure_pursuit_controller` because DWB handles tight spaces and near-obstacle trajectories better. Pure Pursuit follows a lookahead point on the path and struggles when that point lands in an inflated zone — it stops rather than detouring. DWB samples many trajectory candidates and selects the best-scoring one, giving it more recovery options near obstacles.

**A\* over Dijkstra:** NavFn with `use_astar: true` was selected for the global planner. In open grids Dijkstra and A\* produce identical paths, but A\* reaches the goal faster because it uses a heuristic (straight-line distance) to prioritize cells closer to the goal. In a long hospital corridor this makes planning noticeably faster, reducing the BT tick rate violations that occurred when the planner took > 10ms per cycle.

---

## Project Structure

```
hospital_robot_description/
├── config/
│   ├── nav2_params.yaml          # Nav2 full configuration
│   ├── slam_toolbox_params.yaml  # SLAM mapping configuration
│   └── navigate_w_replanning.xml # Custom BT for recovery
├── launch/
│   ├── gz_gazebo.launch.py       # Gazebo + robot + bridge
│   ├── slam.launch.py            # SLAM mapping
│   └── nav2.launch.py            # Nav2 navigation stack
├── maps/
│   ├── hospital_map.pgm          # Saved occupancy grid map
│   └── hospital_map.yaml         # Map metadata (origin, resolution)
├── meshes/
│   ├── chassis.stl
│   ├── wheel_left.stl
│   ├── wheel_right.stl
│   ├── lidar_mount.stl
│   ├── payload_box.stl
│   └── ...
├── scripts/
│   ├── cmd_vel_relay.py          # Relay nav2 cmd_vel_nav → cmd_vel
│   └── recovery_helper.py        # Manual costmap clearing utility
├── urdf/
│   └── robot.urdf.xacro          # Robot description with plugins
├── worlds/
│   └── hospital.world            # Hospital SDF world
├── CMakeLists.txt
└── package.xml
```

---

## Prerequisites

| Dependency | Version |
|---|---|
| ROS 2 | Humble |
| Ignition Gazebo | Fortress |
| Ubuntu | 22.04 |

### Required ROS 2 packages

```bash
sudo apt update && sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard
```

---

## Installation

```bash
# Create workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/bismanroopsingh/Ros-project.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

---

## Quick Start

```bash
# Terminal 1 — Launch Gazebo + robot
ros2 launch hospital_robot_description gz_gazebo.launch.py

# Terminal 2 — Launch Nav2 (waits 20s automatically for robot to spawn)
ros2 launch hospital_robot_description nav2.launch.py

# Terminal 3 — Open RViz
rviz2
```

In RViz:
1. Set **Fixed Frame** to `map`
2. Add **Map** → `/map`
3. Add **Map** → `/global_costmap/costmap`
4. Add **LaserScan** → `/scan`
5. Add **Path** → `/plan`
6. Click **Nav2 Goal** and click a location in the corridor

---

## Running SLAM Mapping

Use this to build a new map of the hospital environment.

```bash
# Terminal 1 — Gazebo + robot
ros2 launch hospital_robot_description gz_gazebo.launch.py

# Terminal 2 — SLAM
ros2 launch hospital_robot_description slam.launch.py

# Terminal 3 — Teleop to drive and map
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Mapping tips
- Drive slowly (20–30% speed)
- Turn slowly — fast turns cause scan-matching drift
- Complete one clean perimeter loop of the corridor
- Do not enter single-doorway rooms (causes loop closure failure)
- Pause 2–3 seconds at each corner before turning

### Save the map
```bash
ros2 run nav2_map_server map_saver_cli -f src/hospital_robot_description/maps/hospital_map
```

This creates `hospital_map.pgm` + `hospital_map.yaml`.

---

## Running Navigation

```bash
# Terminal 1
ros2 launch hospital_robot_description gz_gazebo.launch.py

# Terminal 2 — Nav2 auto-starts after 20s delay
ros2 launch hospital_robot_description nav2.launch.py
```

AMCL automatically initializes the robot pose at spawn `(0.0, -2.9)` — no manual 2D Pose Estimate needed.

---

## Delivery Mission

The primary mission is a two-stop delivery route: **Spawn → Supply Table → Reception Desk**.

This simulates a real hospital workflow where the robot:
1. Starts at its docking/charging position in the main corridor
2. Navigates to the supply table in the reception area to pick up supplies
3. Delivers to the reception desk

### Run the delivery mission

```bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses '{
  poses: [
    {
      header: {frame_id: "map"},
      pose: {
        position: {x: -8.5, y: 8.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    {
      header: {frame_id: "map"},
      pose: {
        position: {x: -8.5, y: 7.5, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    }
  ]
}'
```

### Mission route breakdown

```
SPAWN (0.0, -2.9)
    │
    │  ~8.5m west + 10.9m north through main corridor
    │
STOP 1: Supply Table (-8.5, 8.0)
    │   Robot pauses here (pickup)
    │
    │  ~0.5m south along reception wall
    │
STOP 2: Reception Desk (-8.5, 7.5)
    │   Robot stops here (delivery)
    │
    Mission complete
```

### Expected terminal output on success

```
[bt_navigator]: Begin navigating from (0.00, -2.90) to (-8.50, 8.00)
[controller_server]: Passing new path to controller.
[bt_navigator]: Begin navigating from (-8.50, 8.00) to (-8.50, 7.50)
[controller_server]: Passing new path to controller.
Goal succeeded!
```

---

## Sending Navigation Goals

### Single goal via RViz
Click **Nav2 Goal** button and click anywhere in the white corridor area on the map.

### Single goal via terminal
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{
  pose: {
    header: {frame_id: "map"},
    pose: {
      position: {x: -8.5, y: 8.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}'
```

### Manual costmap clear (use if robot gets stuck)
```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap {}
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap {}
```

---

## Key Locations

These coordinates are confirmed free space in the hospital world:

| Location | x | y | Notes |
|---|---|---|---|
| Spawn / Start | 0.0 | -2.9 | Main corridor — robot home position |
| **Supply Table** | **-8.5** | **8.0** | **Delivery Mission Stop 1** |
| **Reception Desk** | **-8.5** | **7.5** | **Delivery Mission Stop 2** |
| Supply Boxes | -8.5 | 10.5 | Near north wall |
| Nurses Station | 1.0 | -0.5 | In front of desk |
| Charging Dock | -8.5 | -4.5 | Storage bay |
| Shelf Area | -5.5 | -5.0 | Storage corridor |
| Room 1 Doorway | 0.0 | 3.0 | Outside patient room |
| Room 2 Doorway | 6.0 | 3.0 | Outside patient room |

---

## Architecture

```
Mission Goal (RViz / terminal)
         │
  Global Planner — NavFn A*
         │         /plan topic
  Behavior Tree — bt_navigator
         │         recovery: spin → backup → wait
  Local Planner — DWB Controller
         │         /cmd_vel_nav @ 20Hz
  cmd_vel_relay.py
         │         /cmd_vel
  ros_gz_bridge
         │
  Ignition DiffDrive Plugin
         │
  left_wheel_joint + right_wheel_joint
         │
  /odom ──→ EKF (robot_localization) ──→ odom→base_link TF
         │
  /scan  ──→ AMCL ──→ map→odom TF
         │
  Obstacle Layer (local + global costmap)
```

---

## Configuration

### slam_toolbox_params.yaml

| Parameter | Value | Notes |
|---|---|---|
| `resolution` | 0.05 | 5cm per cell |
| `max_laser_range` | 12.0 | Matches sensor max |
| `minimum_travel_distance` | 0.2 | Update after 20cm movement |
| `minimum_travel_heading` | 0.3 | Update after ~17° turn |
| `map_update_interval` | 1.0 | Map refresh rate |

### nav2_params.yaml key values

| Parameter | Value | Notes |
|---|---|---|
| `inflation_radius` | 0.10 | Small — prevents false collision detections |
| `cost_scaling_factor` | 10.0 | Sharp cost falloff |
| `robot_radius` | 0.25 | Conservative estimate |
| `use_astar` | true | Better for long corridor routes |
| `observation_persistence` | 0.0 | Clears dynamic obstacles immediately |
| `max_vel_x` | 0.5 | Linear speed limit |
| `max_vel_theta` | 1.5 | Angular speed limit |
| `simulate_ahead_time` | 1.0 | Reduced — fewer false collision detections |
| `global_costmap update_frequency` | 0.5 | Reduces BT tick rate violations |

---

## Known Issues & Fixes

### Robot stops near obstacles
**Cause:** Inflation radius too large — adjacent obstacle zones merge, surrounding robot.
**Fix:** Reduce `inflation_radius` to `0.10` in both costmaps.

### Second goal fails after first succeeds
**Cause:** `observation_persistence > 0` keeps stale obstacles in costmap from goal 1.
**Fix:** Set `observation_persistence: 0.0` in both costmap scan sections.

### Behavior Tree tick rate exceeded warnings
**Cause:** Planner taking > 10ms per tick due to large global costmap updates.
**Fix:** Reduce `global_costmap update_frequency` to `0.5` and fix spin failure by reducing inflation.

### Spin recovery fails with "Collision Ahead"
**Cause:** Inflation too large — robot appears surrounded even in open space.
**Fix:** Reduce `inflation_radius` and `simulate_ahead_time` in behavior_server.

### SLAM map has rotated/ghost walls
**Cause:** Driving too fast during mapping — scan-matching can't keep up.
**Fix:** Drive at 20–30% teleop speed, turn slowly, complete one clean loop.

### `/scan` shows all zeros
**Cause:** `lidar_link` had a `<collision>` element surrounding the sensor origin.
**Fix:** Remove `<collision>` block from `lidar_link` in URDF — sensor links don't need collision geometry.

### `odom` frame missing from TF tree
**Cause:** EKF node defined but not added to `LaunchDescription` return list.
**Fix:** Assign EKF node to a variable and include it in the return list.

### Map origin mismatch (costmap misaligned)
**Cause:** Robot spawning outside the saved map's southern boundary.
**Fix:** Adjust `origin` y-value in `hospital_map.yaml` to cover spawn position.

### Human actors block navigation path
**Cause:** Actors have collision cylinders that the lidar detects, creating phantom obstacles in nav2 costmap even though the robot can physically pass through them.
**Fix:** Comment out all `<actor>` blocks in `hospital.world` for navigation sessions.

---

## Troubleshooting

### Check TF tree
```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link
```
Expected chain: `map → odom → base_link → lidar_link`

### Check all topics are alive
```bash
ros2 topic list | grep -E "scan|odom|cmd_vel|map|plan"
ros2 topic hz /scan      # expect ~10 Hz
ros2 topic hz /odom      # expect ~10 Hz
```

### Check nav2 node states
```bash
ros2 node list | grep -E "amcl|planner|controller|costmap"
ros2 lifecycle get /local_costmap/local_costmap   # expect: active
ros2 lifecycle get /global_costmap/global_costmap # expect: active
```

### Check AMCL localization
```bash
ros2 topic echo /amcl_pose --once | grep -A3 "position"
# x should be near 0.0, y should be near -2.9 at startup
```

### Manually reset robot pose if AMCL drifts
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {
    pose: {
      position: {x: 0.0, y: -2.9, z: 0.0},
      orientation: {w: 1.0}
    },
    covariance: [0.1,0,0,0,0,0, 0,0.1,0,0,0,0,
                 0,0,0,0,0,0, 0,0,0,0,0,0,
                 0,0,0,0,0,0, 0,0,0,0,0,0.05]
  }
}'
```

---

## License

MIT License — see [LICENSE](LICENSE) for details.

---

## Acknowledgements

- [Nav2 Documentation](https://navigation.ros.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics) — Nav2 tutorial reference
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization)