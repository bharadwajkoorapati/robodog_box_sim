# File-wise changes

- launch/gz_sim_launch.py: replaces scattered timing edits with lifecycle-managed SLAM,
  map/TF readiness gating, mutually exclusive custom/Nav2 operation, saved-map AMCL
  mode, world-name-derived bridge topic, optional NVIDIA offload and RViz.
- launch/rviz_only_launch.py: matching standalone visualization entry point.
- src/navigation_ready.cpp (new): bounded map and TF readiness check.
- src/autonavigator_node.cpp: SensorDataQoS; missing/invalid/stale-data stop;
  target-frame check; front emergency stop; remove false-success obstacle/Y checks;
  initial state timing starts with actual data; reset negative-time state.
- src/target_cube_publisher.cpp: explicit standard headers and publish-rate validation;
  preserves target coordinates and odom-frame convention for the default spawn.
- config/nav2_params.yaml: consistent base frame, conservative radius, lower minimum
  regulated speed, progress checker tuning, sensor timeout, explicit unstamped commands,
  AMCL default initial pose for the supplied spawn.
- config/slam_toolbox_params.yaml: simulation time explicit; interactive mode disabled.
- urdf/ultrasonic_box_bot.urdf.xacro: preserves the corrected upload's ultrasonic FOV,
  range and frame IDs, single LiDAR range declaration, and rotated wheel inertia.
- worlds/empty_us_world.sdf: removes zero-sized target collision and competing Gazebo
  keyboard command publishers; preserves maze, sensors system and target visual.
- rviz/view_bot_rviz_config.rviz: map fixed frame, Nav2 goal and initial-pose tools,
  best-effort scans and transient-local map/description subscriptions.
- CMakeLists.txt: C++17, readiness target, tf2 dependencies, optional installation of
  existing unsupplied utilities and marker script.
- package.xml: dependencies matching the new launch/helper code and Nav2 components.
- README.md and .context.md: current architecture, exact commands, acceptance checklist.

Important: gz.msgs.PointCloudPacked is the correct point-cloud bridge type.
The previous chat's correction explaining this was garbled; this archive uses that
exact type consistently. No runtime test is represented as completed.
