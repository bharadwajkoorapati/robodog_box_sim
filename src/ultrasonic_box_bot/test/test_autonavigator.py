"""autonavigator_node: drives, e-stops on a close obstacle, stops on stale data.

All three behaviours were added as hardening and are timing-sensitive, so they
are checked against the real node over DDS rather than by unit-testing helpers.
"""

import time

from conftest import wait_until

DATA_TIMEOUT_SEC = 0.5
EMERGENCY_STOP_DISTANCE = 0.22


def _scan(distance):
    from sensor_msgs.msg import LaserScan
    msg = LaserScan()
    msg.header.frame_id = 'base_footprint'
    msg.angle_min = -0.26
    msg.angle_max = 0.26
    msg.angle_increment = 0.52
    msg.range_min = 0.02
    msg.range_max = 4.0
    msg.ranges = [distance, distance, distance]
    return msg


def test_drive_estop_and_stale_stop(ros, spawn):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    class Probe(Node):
        def __init__(self):
            super().__init__('navigator_probe')
            self.pubs = {
                'front': self.create_publisher(
                    LaserScan, '/us_front', qos_profile_sensor_data),
                'left': self.create_publisher(
                    LaserScan, '/us_left', qos_profile_sensor_data),
                'right': self.create_publisher(
                    LaserScan, '/us_right', qos_profile_sensor_data),
                'odom': self.create_publisher(Odometry, '/odom', 10),
            }
            self.cmds = []
            self.create_subscription(Twist, '/cmd_vel', self.cb, 50)

        def cb(self, msg):
            self.cmds.append((time.time(), msg.linear.x, msg.angular.z))

        def feed(self, front=1.0):
            stamp = self.get_clock().now().to_msg()
            for name in ('front', 'left', 'right'):
                scan = _scan(front)
                scan.header.stamp = stamp
                self.pubs[name].publish(scan)
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose.orientation.w = 1.0
            self.pubs['odom'].publish(odom)

    spawn('ultrasonic_box_bot', 'autonavigator_node', {
        'cmd_vel_topic': '/cmd_vel',
        'data_timeout_sec': DATA_TIMEOUT_SEC,
        'emergency_stop_distance': EMERGENCY_STOP_DISTANCE,
        'target_frame': 'odom',
    })

    probe = Probe()

    def pump(seconds, front):
        end = time.time() + seconds
        while time.time() < end:
            probe.feed(front)
            rclpy.spin_once(probe, timeout_sec=0.01)
            time.sleep(0.05)

    try:
        # A: clear path -> the FSM commands forward motion.
        pump(1.5, 1.0)
        wait_until(lambda: [c for c in probe.cmds if c[1] > 0.05], 3.0,
                   'navigator never commanded forward motion on clear data')

        # B: obstacle inside the emergency band -> no forward command.
        # The first 0.3 s are a settle window: a command already in flight
        # before the 0.10 m reading reached the subscriber cannot be judged.
        probe.cmds.clear()
        settle_until = time.time() + 0.3
        pump(1.0, 0.10)
        offending = [c for c in probe.cmds
                     if c[0] > settle_until and c[1] > 0.0]
        assert not offending, (
            'drove forward after an obstacle inside the e-stop distance was '
            f'received: {offending}')

        # C: sensor stream stops -> the watchdog commands a stop.
        probe.cmds.clear()
        end = time.time() + 2.0
        while time.time() < end:
            rclpy.spin_once(probe, timeout_sec=0.01)
            time.sleep(0.01)
        wait_until(
            lambda: [c for c in probe.cmds if c[1] == 0.0 and c[2] == 0.0],
            2.0, 'no stop command after the sensor stream went stale')
    finally:
        probe.destroy_node()
