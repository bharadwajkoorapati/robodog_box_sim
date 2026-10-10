# Autonomous Navigation Robot (`ultrasonic_box_bot`)

An autonomous navigation mobile robot built on **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic (`gz-sim`)**. The robot utilizes a differential-drive chassis equipped with four directional ultrasonic rangefinders, a 360° 2D LiDAR, and ROS 2 communication pipelines to perform autonomous exploration, target localization, and navigation.

---

## Features & Highlights

- **Simulation Platform**: Modern Gazebo Harmonic integrated through `ros_gz_sim` and `ros_gz_bridge`.
- **Differential Drive Kinematics**: Complete chassis model with drive wheels, passive caster, and `DiffDrive` odometry publishing.
- **Multi-Sensor Perception**:
  - **4x Ultrasonic Sensors**: Front, back, left, and right directional range sensors.
  - **360° 2D LiDAR**: Planar laser scanning for mapping and obstacle detection.
- **Target Tracking Pipeline**: Dedicated `target_cube_publisher` delivering ground-truth target coordinates (`(X: 4.0, Y: -4.0, Z: 0.125)`) directly to `/target_cube/pose` (`geometry_msgs/msg/PoseStamped`).
- **Reactive Maze Navigation**: Complete state machine navigation loop in `autonavigator_node` utilizing front ultrasonic wall detection, left/right ultrasonic wall following, and odometry heading tracking to maneuver through maze walls and reach the target cube.
- **Modular C++ Nodes**: High-performance `rclcpp` nodes for sensor processing, target monitoring, and motion control.

---

## Incremental Development Roadmap

- [x] **Step 1: Target Cube Location Reader** — Implemented `target_cube_publisher` and `target_cube_listener` to reliably publish and log target position coordinates and orientation.
- [x] **Step 2: Autonomous Navigator Foundation** — Implemented `autonavigator_node` skeleton with target cube subscriber and periodic control callbacks.
- [x] **Step 3: Ultrasonic Wall-Following State Machine** — Implemented end-to-end navigation logic adapted from rover firmware (`CAS_source_TEAM6.ino`) to steer through maze corridors, navigate gaps, and reach the target cube.
- [ ] **Step 4: Dynamic Obstacle & LiDAR Fusion** — Blend real-time 360° LiDAR obstacle mapping for dynamic detour avoidance.

---

## Repository Structure

```
ultrasonic_box_bot/
├── CMakeLists.txt              # Package build configuration
├── package.xml                 # ROS 2 dependencies
├── README.md                   # Public documentation and roadmap
├── launch/
│   ├── gz_sim_launch.py        # Main launch file (Gazebo + RViz + Bridge + Target Publisher)
│   └── rviz_only_launch.py     # RViz-only visualization launch
├── urdf/
│   └── ultrasonic_box_bot.urdf.xacro # Robot description and sensor definition
├── worlds/
│   ├── empty_us_world.sdf      # Main simulation world with maze walls and target cube
│   └── empty_us_world_v2.sdf   # Secondary world layout
├── src/
│   ├── autonavigator_node.cpp    # Ultrasonic wall-following autonomous navigator
│   ├── target_cube_publisher.cpp # Target cube position publisher
│   ├── target_cube_listener.cpp  # Target cube pose reader node (Step 1)
│   ├── lidar_detector.cpp        # 2D LiDAR obstacle detection node
│   └── ultrasonic_listener.cpp   # Ultrasonic parser & emergency stop node
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
    ros-jazzy-xacro
  ```

### Build Instructions

From your ROS 2 workspace root (`~/ros2/ros2_ws`):

```bash
# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Build the package
colcon build --packages-select ultrasonic_box_bot

# Source the workspace overlay
source install/setup.bash
```

---

## Running the System

### 1. Launch the Simulation Environment

Starts Gazebo Harmonic, spawns the robot with all sensors, initializes the ROS-Gazebo bridge, starts the target cube publisher, and opens RViz:

```bash
ros2 launch ultrasonic_box_bot gz_sim_launch.py
```

### 2. Run the Autonomous Wall-Following Navigator

In a new terminal:

```bash
source install/setup.bash
ros2 run ultrasonic_box_bot autonavigator_node
```

**Navigation Sequence in Console**:
```
[INFO] [autonavigator_node]: Autonavigator Node initialized. Starting state machine...
[INFO] [autonavigator_node]: [State Transition] -> TURN_RIGHT_WALL_1 (X: 0.45, Y: 0.00, Yaw: 0.0 deg)
[INFO] [autonavigator_node]: [State Transition] -> FOLLOW_WALL_1 (X: 0.45, Y: 0.00, Yaw: -89.8 deg)
[INFO] [autonavigator_node]: [State Transition] -> CLEAR_CORNER_1 (X: 0.47, Y: -3.16, Yaw: -90.0 deg)
[INFO] [autonavigator_node]: [State Transition] -> TURN_LEFT_GAP_1 (X: 0.47, Y: -3.55, Yaw: -90.0 deg)
[INFO] [autonavigator_node]: [State Transition] -> FORWARD_TO_WALL_2 (X: 0.47, Y: -3.55, Yaw: 0.2 deg)
[INFO] [autonavigator_node]: [State Transition] -> TURN_LEFT_WALL_2 (X: 2.45, Y: -3.55, Yaw: 0.0 deg)
[INFO] [autonavigator_node]: [State Transition] -> FOLLOW_WALL_2 (X: 2.45, Y: -3.55, Yaw: 89.9 deg)
[INFO] [autonavigator_node]: [State Transition] -> CLEAR_CORNER_2 (X: 2.45, Y: 3.16, Yaw: 90.0 deg)
[INFO] [autonavigator_node]: [State Transition] -> TURN_RIGHT_GAP_2 (X: 2.45, Y: 3.55, Yaw: 90.0 deg)
[INFO] [autonavigator_node]: [State Transition] -> FORWARD_TO_PERIMETER (X: 2.45, Y: 3.55, Yaw: -0.1 deg)
[INFO] [autonavigator_node]: [State Transition] -> TURN_RIGHT_PASSAGE (X: 4.45, Y: 3.55, Yaw: 0.0 deg)
[INFO] [autonavigator_node]: [State Transition] -> FOLLOW_PASSAGE_TO_TARGET (X: 4.45, Y: 3.55, Yaw: -89.9 deg)
[INFO] [autonavigator_node]: >>> TARGET REACHED! Mission Accomplished at (X: 4.02, Y: -3.98) <<<
```

### 3. Monitoring & Helper Nodes

- **Target Cube Reader**:
  ```bash
  ros2 run ultrasonic_box_bot target_cube_listener
  ```
- **LiDAR Obstacle Detector**:
  ```bash
  ros2 run ultrasonic_box_bot lidar_detector
  ```
- **Ultrasonic Front Range Listener**:
  ```bash
  ros2 run ultrasonic_box_bot ultrasonic_listener
  ```

---

## License

This project is licensed under the Apache 2.0 License.
