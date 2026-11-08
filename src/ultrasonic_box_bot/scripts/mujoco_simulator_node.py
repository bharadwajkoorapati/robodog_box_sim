#!/usr/bin/env python3
"""MuJoCo Simulator Node for ultrasonic_box_bot."""

import math
import os
import sys
import threading
import time

import mujoco
try:
    import mujoco.viewer
except ImportError:
    pass
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class MujocoSimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__('mujoco_simulator_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_safe')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('wheel_radius', 0.06)
        self.declare_parameter('wheel_separation', 0.34)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('scan_rate_hz', 10.0)
        self.declare_parameter('us_rate_hz', 10.0)
        self.declare_parameter('odom_rate_hz', 20.0)
        self.declare_parameter('headless', True)
        self.declare_parameter('real_time_factor', 1.0)
        self.declare_parameter('lidar_samples', 361)
        self.declare_parameter('lidar_range_min', 0.08)
        self.declare_parameter('lidar_range_max', 10.0)
        self.declare_parameter('lidar_fov_deg', 360.0)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            try:
                pkg_share = get_package_share_directory('ultrasonic_box_bot')
                model_path = os.path.join(pkg_share, 'worlds', 'mujoco', 'slam_world.xml')
            except PackageNotFoundError:
                model_path = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'worlds', 'mujoco', 'slam_world.xml'
                )

        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"MuJoCo model not found: '{model_path}'")

        self.get_logger().info(f"Loading MuJoCo model: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.scan_rate = float(self.get_parameter('scan_rate_hz').value)
        self.us_rate = float(self.get_parameter('us_rate_hz').value)
        self.odom_rate = float(self.get_parameter('odom_rate_hz').value)
        self.headless = bool(self.get_parameter('headless').value)
        self.real_time_factor = float(self.get_parameter('real_time_factor').value)

        self.lidar_samples = int(self.get_parameter('lidar_samples').value)
        self.lidar_range_min = float(self.get_parameter('lidar_range_min').value)
        self.lidar_range_max = float(self.get_parameter('lidar_range_max').value)
        self.lidar_angles = np.linspace(-np.pi, np.pi, self.lidar_samples)
        self.lidar_local_vecs = np.column_stack([
            np.cos(self.lidar_angles),
            np.sin(self.lidar_angles),
            np.zeros_like(self.lidar_angles)
        ])

        # 5 rays spanning 30 deg FOV
        self.us_fov = math.radians(30.0)
        self.us_samples = 5
        self.us_angles = np.linspace(-self.us_fov / 2.0, self.us_fov / 2.0, self.us_samples)
        self.us_local_vecs = np.column_stack([
            np.cos(self.us_angles),
            np.sin(self.us_angles),
            np.zeros_like(self.us_angles)
        ])
        self.us_range_min = 0.02
        self.us_range_max = 4.0

        self.robot_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        self.lidar_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'lidar')
        self.us_site_ids = {
            'us_front': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'us_front'),
            'us_back': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'us_back'),
            'us_left': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'us_left'),
            'us_right': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'us_right'),
        }

        self.left_actuator_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'left_wheel_actuator'
        )
        self.right_actuator_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'right_wheel_actuator'
        )
        self.left_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, 'left_wheel_joint'
        )
        self.right_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, 'right_wheel_joint'
        )

        self._geomid = np.zeros(1, dtype=np.int32)

        self.target_v = 0.0
        self.target_w = 0.0
        self.last_cmd_sim_time = 0.0

        self.last_scan_sim_time = -1.0
        self.last_us_sim_time = -1.0
        self.last_odom_sim_time = -1.0

        self.wall_start_time = time.perf_counter()
        self.sim_start_time = self.data.time

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10
        )
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.scan_pub = self.create_publisher(
            LaserScan, self.get_parameter('scan_topic').value, qos_profile_sensor_data
        )

        self.us_pubs = {
            name: self.create_publisher(LaserScan, f'/{name}', qos_profile_sensor_data)
            for name in self.us_site_ids
        }

        self.tf_broadcaster = TransformBroadcaster(self)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_vel_sub = self.create_subscription(
            Twist, cmd_vel_topic, self._cmd_vel_callback, 10
        )

        self.viewer = None
        if not self.headless and 'DISPLAY' in os.environ and hasattr(mujoco, 'viewer'):
            try:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            except Exception as exc:
                self.get_logger().warn(f"Failed to start viewer: {exc}. Running headless.")

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self._sim_step)
        self.get_logger().info(
            f"MuJoCo simulator started (dt={self.model.opt.timestep}s, {self.publish_rate}Hz)"
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.target_v = float(msg.linear.x)
        self.target_w = float(msg.angular.z)
        self.last_cmd_sim_time = self.data.time

    def _sim_step(self) -> None:
        current_sim_time = self.data.time

        if (current_sim_time - self.last_cmd_sim_time) > self.cmd_vel_timeout:
            v_cmd, w_cmd = 0.0, 0.0
        else:
            v_cmd, w_cmd = self.target_v, self.target_w

        # Inverse kinematics
        omega_left = (v_cmd - 0.5 * w_cmd * self.wheel_separation) / self.wheel_radius
        omega_right = (v_cmd + 0.5 * w_cmd * self.wheel_separation) / self.wheel_radius

        if self.left_actuator_id >= 0:
            self.data.ctrl[self.left_actuator_id] = omega_left
        if self.right_actuator_id >= 0:
            self.data.ctrl[self.right_actuator_id] = omega_right

        # Sync sim time to wall clock
        elapsed_wall = time.perf_counter() - self.wall_start_time
        desired_sim_time = self.sim_start_time + (elapsed_wall * self.real_time_factor)

        # Avoid spiral of death on freeze or breakpoint
        if (desired_sim_time - self.data.time) > 0.2:
            self.wall_start_time = time.perf_counter()
            self.sim_start_time = self.data.time
            desired_sim_time = self.data.time + (1.0 / self.publish_rate)

        while self.data.time < desired_sim_time:
            mujoco.mj_step(self.model, self.data)

        sim_time = self.data.time

        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

        clock_msg = Clock()
        sec = int(sim_time)
        nanosec = int((sim_time - sec) * 1e9)
        clock_msg.clock.sec = sec
        clock_msg.clock.nanosec = nanosec
        self.clock_pub.publish(clock_msg)

        ros_stamp = Time(seconds=sec, nanoseconds=nanosec).to_msg()

        if (sim_time - self.last_odom_sim_time) >= (1.0 / self.odom_rate):
            self.last_odom_sim_time = sim_time
            self._publish_odometry(ros_stamp, v_cmd, w_cmd)
            self._publish_joint_states(ros_stamp)

        if (sim_time - self.last_scan_sim_time) >= (1.0 / self.scan_rate):
            self.last_scan_sim_time = sim_time
            self._publish_lidar_scan(ros_stamp)

        if (sim_time - self.last_us_sim_time) >= (1.0 / self.us_rate):
            self.last_us_sim_time = sim_time
            self._publish_ultrasonic_scans(ros_stamp)

    def _publish_odometry(self, stamp, v_cmd: float, w_cmd: float) -> None:
        pos = self.data.qpos[0:3]
        quat_mj = self.data.qpos[3:7]  # [w, x, y, z]

        qw, qx, qy, qz = quat_mj[0], quat_mj[1], quat_mj[2], quat_mj[3]
        yaw = quaternion_to_yaw(qx, qy, qz, qw)
        qx_2d, qy_2d, qz_2d, qw_2d = euler_to_quaternion(0.0, 0.0, yaw)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx_2d
        t.transform.rotation.y = qy_2d
        t.transform.rotation.z = qz_2d
        t.transform.rotation.w = qw_2d
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx_2d
        odom.pose.pose.orientation.y = qy_2d
        odom.pose.pose.orientation.z = qz_2d
        odom.pose.pose.orientation.w = qw_2d

        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.001

        odom.twist.twist.linear.x = v_cmd
        odom.twist.twist.angular.z = w_cmd
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[35] = 0.001

        self.odom_pub.publish(odom)

    def _publish_joint_states(self, stamp) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = ['left_wheel_joint', 'right_wheel_joint']

        qpos_left_idx = self.model.jnt_qposadr[self.left_joint_id]
        qpos_right_idx = self.model.jnt_qposadr[self.right_joint_id]
        qvel_left_idx = self.model.jnt_dofadr[self.left_joint_id]
        qvel_right_idx = self.model.jnt_dofadr[self.right_joint_id]

        msg.position = [
            float(self.data.qpos[qpos_left_idx]),
            float(self.data.qpos[qpos_right_idx]),
        ]
        msg.velocity = [
            float(self.data.qvel[qvel_left_idx]),
            float(self.data.qvel[qvel_right_idx]),
        ]
        self.joint_state_pub.publish(msg)

    def _publish_lidar_scan(self, stamp) -> None:
        if self.lidar_site_id < 0:
            return

        origin = self.data.site_xpos[self.lidar_site_id]
        rot_mat = self.data.site_xmat[self.lidar_site_id].reshape(3, 3)
        world_vecs = self.lidar_local_vecs @ rot_mat.T

        ranges = []
        for vec in world_vecs:
            dist = mujoco.mj_ray(
                self.model,
                self.data,
                origin,
                vec,
                None,
                1,
                self.robot_body_id,
                self._geomid,
            )
            if dist < 0 or dist > self.lidar_range_max:
                ranges.append(float('inf'))
            elif dist < self.lidar_range_min:
                ranges.append(0.0)
            else:
                ranges.append(float(dist))

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = float(self.lidar_angles[0])
        scan.angle_max = float(self.lidar_angles[-1])
        scan.angle_increment = float(self.lidar_angles[1] - self.lidar_angles[0])
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.scan_rate
        scan.range_min = self.lidar_range_min
        scan.range_max = self.lidar_range_max
        scan.ranges = ranges

        self.scan_pub.publish(scan)

    def _publish_ultrasonic_scans(self, stamp) -> None:
        for name, site_id in self.us_site_ids.items():
            if site_id < 0:
                continue

            origin = self.data.site_xpos[site_id]
            rot_mat = self.data.site_xmat[site_id].reshape(3, 3)
            world_vecs = self.us_local_vecs @ rot_mat.T

            min_dist = self.us_range_max
            for vec in world_vecs:
                dist = mujoco.mj_ray(
                    self.model,
                    self.data,
                    origin,
                    vec,
                    None,
                    1,
                    self.robot_body_id,
                    self._geomid,
                )
                if 0.0 <= dist < min_dist:
                    min_dist = dist

            clamped_dist = max(self.us_range_min, min(self.us_range_max, min_dist))

            us_msg = LaserScan()
            us_msg.header.stamp = stamp
            us_msg.header.frame_id = f'{name}_link'
            us_msg.angle_min = float(self.us_angles[0])
            us_msg.angle_max = float(self.us_angles[-1])
            us_msg.angle_increment = float(self.us_angles[1] - self.us_angles[0])
            us_msg.time_increment = 0.0
            us_msg.scan_time = 1.0 / self.us_rate
            us_msg.range_min = self.us_range_min
            us_msg.range_max = self.us_range_max
            us_msg.ranges = [float(clamped_dist)] * self.us_samples

            self.us_pubs[name].publish(us_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MujocoSimulatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.viewer is not None and node.viewer.is_running():
            node.viewer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
