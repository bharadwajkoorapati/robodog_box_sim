# Autonomous Navigation Robot (`ultrasonic_box_bot`)

An autonomous navigation mobile robot built on **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic (`gz-sim`)**. The robot has a differential-drive chassis, four directional ultrasonic rangefinders and a 360° 2D LiDAR, and runs a complete SLAM → Nav2 stack alongside a scripted wall-following state machine.

---

## Features & Highlights

- **Simulation platform**: Gazebo Harmonic integrated through `ros_gz_sim` and `ros_gz_bridge`.
- **Differential drive**: chassis model with drive wheels, passive caster and `DiffDrive` odometry.
- **Multi-sensor perception**:
  - **4x ultrasonic sensors**: front, back, left, right range sensors.
  - **360° 2D LiDAR**: planar laser scan for mapping and obstacle detection.
- **Target tracking pipeline**: `target_cube_publisher` publishes the ground-truth target (`X: 4.0, Y: -4.0, Z: 0.125`) on `/target_cube/pose` (`geometry_msgs/msg/PoseStamped`).
- **Nav2 goal navigation**: `target_nav2_client` transforms the target pose into the `map` frame and sends a `NavigateToPose` goal — so the target position actually drives Nav2 instead of only being logged.
- **Coordinate goals from the command line**: `nav_goal_sender 4.0 4.0` sends a single `NavigateToPose` goal and waits for the result, replacing the long `ros2 action send_goal` incantation.
- **Arena-sized planning bounds**: `map_padder` republishes `/map` inside a fixed 12 × 12 m window, so goals in not-yet-explored parts of the arena are plannable (`allow_unknown`) and the global costmap stops resizing as SLAM grows.
- **Always live mapping**: every launch builds its own map with SLAM and navigates on it; no map is written to disk or loaded from a previous session. Goals in unexplored space are planned as unknown and replanned as the map fills in.
- **Reactive maze navigation**: `autonavigator_node`, a 14-state machine that wall-follows through the fixed maze using ultrasonic ranges and odometry heading.
- **Mapping and localization**: SLAM Toolbox builds a live map; a saved map can be reused with `map_server` + AMCL (`localization:=true`).
- **Command safety**: `cmd_vel_watchdog` injects a zero velocity when command streams stop, so the Gazebo `DiffDrive` plugin never keeps applying a stale Twist.
- **Dependency-aware startup**: lifecycle transitions and readiness gates (`sensor_ready`, `navigation_ready`) instead of fixed sleeps.

---

## Incremental Development Roadmap

- [x] **Step 1: Target cube location reader** — `target_cube_publisher` / `target_cube_listener`.
- [x] **Step 2: Autonomous navigator foundation** — target subscriber + periodic control loop.
- [x] **Step 3: Ultrasonic wall-following state machine** — route adapted from the rover firmware (`CAS_source_TEAM6.ino`).
- [x] **Step 4: SLAM Toolbox integration** — live `/map` and `map -> odom`, gated on sensor readiness.
- [x] **Step 5: Nav2 integration** — costmaps, `NavfnPlanner`, `RegulatedPurePursuitController`.
- [x] **Step 6: Target → Nav2 goal client** — `target_nav2_client` sends `NavigateToPose` goals.
- [x] **Step 7: Map saving + saved-map localization mode** — `save_map_launch.py` and `localization:=true` (map server + AMCL).
- [ ] **Step 8: Runtime verification** — drive the maze, confirm map quality, goal success, and localization-mode repeatability.
- [x] **Step 8a: Arena-sized costmap bounds + x/y goal CLI** — `map_padder` (fixed window, no costmap resizing) and `nav_goal_sender`.
- [ ] **Step 9: Dynamic obstacle / LiDAR fusion** — blend 360° LiDAR for detours in the reactive controller.

---

## Repository Structure

```
ultrasonic_box_bot/
├── CMakeLists.txt              # Package build configuration
├── package.xml                 # ROS 2 dependencies
├── README.md                   # Public documentation
├── launch/
│   ├── gz_sim_launch.py        # Master launch (Gazebo + bridge + SLAM/AMCL + Nav2 + RViz)
│   ├── save_map_launch.py      # Persist the live SLAM map (.yaml + .pgm)
│   └── rviz_only_launch.py     # RViz-only visualization launch
├── config/
│   ├── nav2_params.yaml        # Nav2 + AMCL + map server parameters
│   └── slam_toolbox_params.yaml
├── urdf/
│   └── ultrasonic_box_bot.urdf.xacro # Robot description and sensors
├── worlds/
│   ├── empty_us_world.sdf      # Main world (maze walls + target cube)
│   └── empty_us_world_v2.sdf   # Secondary world layout
├── src/
│   ├── autonavigator_node.cpp    # Ultrasonic wall-following FSM
│   ├── target_cube_publisher.cpp # Ground-truth target pose publisher
│   ├── target_nav2_client.cpp    # Target pose -> NavigateToPose goal
│   ├── nav_goal_sender.cpp       # CLI x y [yaw] -> one NavigateToPose goal
│   ├── map_padder.cpp            # /map -> fixed-window /map_padded for the global costmap
│   ├── target_cube_listener.cpp  # Target pose reader / logger
│   ├── lidar_detector.cpp        # 2D LiDAR obstacle detection
│   ├── ultrasonic_listener.cpp   # Ultrasonic parser & emergency stop demo
│   ├── sensor_ready.cpp          # Gate: /scan + /odom + odom TF live
│   ├── navigation_ready.cpp      # Gate: /map + map -> base_footprint live
│   └── cmd_vel_watchdog.cpp      # /cmd_vel -> /cmd_vel_safe, stale-stop
├── extra/
│   └── wall_marker_publisher.py  # RViz marker utility
└── rviz/
    └── view_bot_rviz_config.rviz # Default RViz display configuration
```

---

## Getting Started

### Prerequisites

- **Ubuntu 24.04 LTS** (or compatible Linux distribution)
- **ROS 2 Jazzy Jalisco**
- **Gazebo Harmonic** (`gz-sim`)
- Core packages:
  ```bash
  sudo apt-get update
  sudo apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup
  ```

### Build Instructions

From your ROS 2 workspace root (`~/ros2/ros2_ws`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ultrasonic_box_bot
source install/setup.bash
```

---

## Running the System

### 1. Launch the simulation (mapping + Nav2)

```bash
ros2 launch ultrasonic_box_bot gz_sim_launch.py
```

Startup order: Gazebo + bridge + robot state publisher → `sensor_ready` exits when `/scan`, `/odom` and `odom -> base_footprint` are live → SLAM is configured and activated → Nav2 starts 3 s after SLAM is `active` → `target_nav2_client` sends the goal for `/target_cube/pose`.

Every launch starts with an empty map. SLAM builds it while the robot drives and
`map_padder` feeds a fixed arena-sized view of it to the global costmap, so a
goal in a part of the arena that has not been seen yet is still accepted and
planned through unknown space. Nothing is persisted: stop the launch and the
map is gone, and the next launch maps again from scratch.

### 2. Drive to coordinates

```bash
# Set the mission target from the launch, then let target_nav2_client drive there
ros2 launch ultrasonic_box_bot gz_sim_launch.py target_x:=4.0 target_y:=4.0

# Or send one goal on demand (x y [yaw]); no launch change needed
ros2 run ultrasonic_box_bot nav_goal_sender 4.0 4.0
ros2 run ultrasonic_box_bot nav_goal_sender --ros-args -p x:=4.0 -p y:=4.0 -p yaw:=1.57
```

The goal is sent in the `map` frame. Unexplored arena cells are unknown and the
planner has `allow_unknown: true`, so the robot heads for the goal and replans
as the map fills in. Goals inside the arena but behind a wall rely on the LiDAR
seeing that wall first (`obstacle_max_range: 8.0`). `nav_goal_sender` exits 0
when the goal succeeds, 1 when it aborts/cancels/times out, 2 on bad usage.

### 3. Launch the scripted wall-following FSM instead of Nav2

```bash
ros2 launch ultrasonic_box_bot gz_sim_launch.py use_autonav:=true
```

The FSM and Nav2 both own `/cmd_vel`, so they are mutually exclusive.

### 4. Optional: load an externally supplied map (not part of the normal flow)

The default flow never saves or loads a map. If you deliberately want to drive
on a map you produced outside this workflow:

```bash
ros2 launch ultrasonic_box_bot gz_sim_launch.py \
  localization:=true map:=/path/to/map.yaml
```

AMCL starts from the origin initial pose in `nav2_params.yaml`; move it with
RViz's **2D Pose Estimate** if the robot does not spawn there. Never run SLAM
and AMCL together — both would publish `map -> odom`.

(`save_map_launch.py` still exists as a manual tool, but the launch never calls
it and never writes a map on its own.)

### 5. Headless / no RViz

```bash
ros2 launch ultrasonic_box_bot gz_sim_launch.py use_rviz:=false
```

### Monitoring & helper nodes

```bash
ros2 run ultrasonic_box_bot target_cube_listener   # log the target pose
ros2 run ultrasonic_box_bot target_nav2_client     # only if the launch did not start it
ros2 run ultrasonic_box_bot lidar_detector         # LiDAR obstacle log
ros2 run ultrasonic_box_bot nav_goal_sender 1.0 0.0 # one-shot goal (map frame)
ros2 run ultrasonic_box_bot ultrasonic_listener    # front range + emergency stop demo
```

### Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `use_autonav` | `false` | Run the scripted FSM instead of Nav2 |
| `localization` | `false` | `false` always builds the map live; `true` loads `map:=` instead |
| `map` | `''` | Map YAML, only used by `localization:=true` |
| `use_rviz` | `true` | Start RViz2 |
| `use_nav2_goal_client` | `true` | Forward `/target_cube/pose` to `NavigateToPose` |
| `cmd_vel_timeout` | `0.5` | Silence (s) before the watchdog commands a stop |
| `target_x` | `4.0` | Target x (m) published on `/target_cube/pose` |
| `target_y` | `-4.0` | Target y (m) published on `/target_cube/pose` |

### Tests

```bash
colcon test --packages-select ultrasonic_box_bot --event-handlers console_direct+
colcon test-result --verbose
```

The tests start the installed nodes as real processes against a throwaway
`ROS_DOMAIN_ID`, so they need no simulator:

| Test | Checks |
|---|---|
| `test_cmd_vel_watchdog.py` | `/cmd_vel` is forwarded to `/cmd_vel_safe`; a zero Twist follows `timeout_sec` of silence |
| `test_target_nav2_client.py` | one `NavigateToPose` goal per target (not one per 10 Hz message), transformed `odom -> map` |
| `test_autonavigator.py` | drives on clear data, refuses to drive with an obstacle inside the e-stop band, stops when sensor data goes stale |
| `test_map_padder.py` | the padded grid covers `(4, 4)` (so the goal is inside the costmap), real cells are preserved and padding is unknown |
| `test_nav_goal_sender.py` | positional `x y` produces exactly one goal in the `map` frame; missing coordinates exit 2 |

---

## License

This project is licensed under the Apache 2.0 License.
