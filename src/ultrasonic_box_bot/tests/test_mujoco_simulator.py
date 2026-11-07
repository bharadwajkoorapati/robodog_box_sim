#!/usr/bin/env python3
"""Unit tests for the MuJoCo simulator node and model for ultrasonic_box_bot."""

import math
import os
import sys
import unittest
import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

# Ensure package root is in sys.path
ws_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ws_path not in sys.path:
    sys.path.insert(0, ws_path)

import rclpy
from geometry_msgs.msg import Twist
from scripts.mujoco_simulator_node import (
    MujocoSimulatorNode,
    euler_to_quaternion,
    quaternion_to_yaw,
)


class TestMujocoSimulator(unittest.TestCase):
    """Test suite verifying MuJoCo model physics, kinematics, and ROS node."""

    @classmethod
    def setUpClass(cls):
        cls.ws_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.model_path = os.path.join(cls.ws_path, 'worlds', 'mujoco', 'slam_world.xml')
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_model_loading_and_structure(self):
        """Verify MJCF XML file loads and contains all expected physical elements."""
        self.assertTrue(os.path.isfile(self.model_path), f"File missing: {self.model_path}")
        m = mujoco.MjModel.from_xml_path(self.model_path)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)

        # Check required bodies
        body_names = [m.body(i).name for i in range(m.nbody)]
        self.assertIn('base_link', body_names)
        self.assertIn('left_wheel_link', body_names)
        self.assertIn('right_wheel_link', body_names)

        # Check required geoms
        geom_names = [m.geom(i).name for i in range(m.ngeom)]
        self.assertIn('ground_plane', geom_names)
        self.assertIn('maze_wall_1', geom_names)
        self.assertIn('maze_wall_2', geom_names)
        self.assertIn('chassis', geom_names)
        self.assertIn('caster_wheel', geom_names)
        self.assertIn('left_wheel', geom_names)
        self.assertIn('right_wheel', geom_names)

        # Check actuators
        actuator_names = [m.actuator(i).name for i in range(m.nu)]
        self.assertIn('left_wheel_actuator', actuator_names)
        self.assertIn('right_wheel_actuator', actuator_names)

        # Check sensor sites
        site_names = [m.site(i).name for i in range(m.nsite)]
        self.assertIn('lidar', site_names)
        self.assertIn('us_front', site_names)
        self.assertIn('us_back', site_names)
        self.assertIn('us_left', site_names)
        self.assertIn('us_right', site_names)

    def test_diff_drive_forward_step(self):
        """Verify differential drive forward velocity commands produce forward translation."""
        m = mujoco.MjModel.from_xml_path(self.model_path)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)

        init_x = float(d.qpos[0])

        # Command wheel speed of 5.0 rad/s (~0.3 m/s) on both wheels
        left_act_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, 'left_wheel_actuator')
        right_act_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, 'right_wheel_actuator')
        d.ctrl[left_act_id] = 5.0
        d.ctrl[right_act_id] = 5.0

        for _ in range(100):  # 0.5s of simulation
            mujoco.mj_step(m, d)

        final_x = float(d.qpos[0])
        self.assertGreater(final_x, init_x, "Robot should translate forward along +X")

    def test_lidar_wall_detection(self):
        """Verify 2D LiDAR raycast correctly detects the front obstacle wall at ~0.95m."""
        m = mujoco.MjModel.from_xml_path(self.model_path)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)

        lidar_site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'lidar')
        origin = d.site_xpos[lidar_site_id]
        rot_mat = d.site_xmat[lidar_site_id].reshape(3, 3)

        # Shoot ray straight forward (+X in lidar frame)
        vec_local = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        vec_world = rot_mat @ vec_local
        geomid = np.zeros(1, dtype=np.int32)
        robot_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'base_link')

        dist = mujoco.mj_ray(m, d, origin, vec_world, None, 1, robot_body_id, geomid)

        # Maze wall 1 is at X=1.0m, thickness=0.1m (half-extent 0.05m -> face at X=0.95m)
        # Robot is at X=0.0m. Lidar origin is at X=0.0m. Expected distance is 0.95m.
        self.assertAlmostEqual(dist, 0.95, delta=0.05)
        hit_geom_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, geomid[0])
        self.assertEqual(hit_geom_name, 'maze_wall_1')

    def test_quaternion_yaw_conversion(self):
        """Verify Euler to Quaternion and Quaternion to Yaw math round-tripping."""
        test_yaws = [-2.5, -1.57, -0.5, 0.0, 0.78, 1.57, 3.14]
        for yaw in test_yaws:
            qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)
            recovered_yaw = quaternion_to_yaw(qx, qy, qz, qw)
            self.assertAlmostEqual(yaw, recovered_yaw, places=4)

    def test_node_instantiation(self):
        """Verify MujocoSimulatorNode initializes cleanly with ROS 2 interfaces."""
        node = MujocoSimulatorNode()
        self.assertIsNotNone(node)
        self.assertIsNotNone(node.clock_pub)
        self.assertIsNotNone(node.odom_pub)
        self.assertIsNotNone(node.scan_pub)
        self.assertEqual(len(node.us_pubs), 4)

        # Simulate one step
        node._sim_step()
        self.assertGreater(node.data.time, 0.0)

        node.destroy_node()


if __name__ == '__main__':
    unittest.main()

