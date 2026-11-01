# File-wise changes

- config/slam_toolbox_params.yaml: stricter scan matching (0.1 -> 0.35), shorter
  link distance, larger minimum travel step, loop closing disabled. Prevents the
  mid-drive `map -> odom` rewrites that made the robot "forget where it is".
- config/nav2_params.yaml: planner frequency 20 -> 5 Hz, AMCL max range 9 m,
  static-layer footprint clearing explicit.
- launch/gz_sim_launch.py: `localization` stays `false` by default - every run
  builds its own map with SLAM and navigates on it. No map is written or loaded
  automatically (`save_map_launch.py` remains a manual tool only).

- src/map_padder.cpp (new): republishes `/map` as `/map_padded` inside a fixed
  12 x 12 m arena window. The global costmap's StaticLayer sizes itself from the
  map, so raw SLAM output made goals in unexplored areas fail with "outside
  bounds" and reallocated the costmap on every map growth. Padded cells are
  unknown, which the planner traverses because `allow_unknown: true`.
- src/nav_goal_sender.cpp (new): one-shot `NavigateToPose` client taking
  `x y [yaw]` positionally or via parameters, waiting for the result and
  exiting 0/1/2 - a short replacement for `ros2 action send_goal`.
- config/nav2_params.yaml: global `static_layer.map_topic: /map_padded`;
  stable costmap timing (`transform_tolerance` 5.0 -> 0.5/0.3, global update
  2 Hz, no forced full costmap); local costmap `ObstacleLayer` instead of the
  voxel grid and 4 x 4 m window; global obstacle range raised to 8 m so maze
  walls are planned around instead of discovered at 2.5 m.
- launch/gz_sim_launch.py: starts `map_padder`; adds `target_x`/`target_y`
  arguments for the target publisher.
- test/test_map_padder.py, test/test_nav_goal_sender.py (new): padded grid
  covers the arena and preserves real cells; CLI sends exactly one goal in the
  `map` frame and reports usage errors.

- src/target_nav2_client.cpp: holds the `tf2_ros::TransformListener` in a
  `std::unique_ptr` instead of a `SharedPtr` member (the class is non-copyable,
  non-movable and has no `SharedPtr` typedef), and keeps its spin thread enabled
  because `to_global_frame()` performs timed lookups.
- test/ (new): node-level pytest suite registered with `ament_add_pytest_test`
  covering the watchdog, the target -> NavigateToPose client and the navigator's
  e-stop/stale-data stops; runs under `colcon test` without a simulator.
- CMakeLists.txt / package.xml: register the tests and their test dependencies.
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

- src/sensor_ready.cpp (new): readiness gate for /scan, /odom and the
  odom -> base_footprint transform; exits 0 when sensors are live.
- launch/gz_sim_launch.py: SLAM is configured from sensor_ready's exit event
  instead of a fixed 8 s timer, removing the cold-start race where SLAM
  activated with an empty TF buffer and never produced /map.
- config/slam_toolbox_params.yaml: scan_queue_size 10 (default 1 dropped scans
  while TF caught up), map_update_interval 1.0, travel gate lowered to 0.15.
- config/nav2_params.yaml: global costmap now uses a StaticLayer sized from the
  SLAM map with rolling_window disabled, so the map is retained instead of
  being re-centred on the robot; removed the inert static_layer block from the
  local costmap.
- rviz/view_bot_rviz_config.rviz: Map display uses Transient Local durability
  and the fixed frame is map, so the latched map is shown on first receipt.
- CMakeLists.txt / package.xml: build and install sensor_ready; tf2_ros is now
  a build dependency.

Important: gz.msgs.PointCloudPacked is the correct point-cloud bridge type.
The previous chat's correction explaining this was garbled; this archive uses that
exact type consistently. No runtime test is represented as completed.

## 2026-10-21: Goals, localization mode and command safety

- src/target_nav2_client.cpp (new): closes the gap between `/target_cube/pose`
  and Nav2. Transforms the target from its own frame into `map` with tf2, sends
  one `NavigateToPose` goal per target change (deduplicating the 10 Hz
  publisher), logs accepted/aborted/canceled results, and optionally cancels an
  in-flight goal when the target moves.
- src/cmd_vel_watchdog.cpp (new): forwards `/cmd_vel` to `/cmd_vel_safe` and
  republishes a zero Twist when the stream goes stale, so an aborted goal or a
  dead controller no longer leaves the DiffDrive plugin applying its last
  command. The bridge now consumes `/cmd_vel_safe`; the watchdog is the single
  owner of that topic.
- src/autonavigator_node.cpp: SensorDataQoS on the three range topics; per-stream
  stale-data stop (`data_timeout_sec`); target-frame check (`target_frame`,
  default `odom`) instead of silently mixing frames; front emergency stop
  (`emergency_stop_distance`); clock-reset-safe state timing.
- launch/gz_sim_launch.py: new `localization`, `map`, `use_rviz`,
  `use_nav2_goal_client` and `cmd_vel_timeout` arguments; saved-map AMCL branch
  (`navigation_launch` gated on `navigation_ready`); target goal client; command
  watchdog; contradictory-argument validation fails fast.
- launch/save_map_launch.py (new): one-command `map_saver_cli` wrapper with sim
  time and transient-local map subscription.
- launch/rviz_only_launch.py: fixed the stale `view_bot.rviz` path (actual file
  is `view_bot_rviz_config.rviz`) and added `use_sim_time`.
- config/nav2_params.yaml: AMCL `set_initial_pose` + origin initial pose so
  localization mode publishes `map -> odom` without a manual RViz click.
- config/slam_toolbox_params.yaml: interactive mode disabled (unused, noisy in
  headless runs).
- CMakeLists.txt / package.xml: build `navigation_ready`, `target_nav2_client`
  and `cmd_vel_watchdog`; add `rclcpp_action`, `tf2_geometry_msgs` and the Nav2
  server dependencies used by the new launch paths.
- README.md and .context.md: new run modes, commands and remaining verification
  checklist.
