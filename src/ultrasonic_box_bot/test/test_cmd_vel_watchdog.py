"""cmd_vel_watchdog: forward commands, then stop the robot when they stop.

Without this node Gazebo's DiffDrive keeps applying the last Twist forever, so
the failure mode being tested is "robot keeps rolling after the controller dies".
"""

import time

from geometry_msgs.msg import Twist


TIMEOUT_SEC = 0.3


def test_forwards_commands_and_stops_on_silence(ros, spawn):
    import rclpy
    from rclpy.node import Node

    spawn('ultrasonic_box_bot', 'cmd_vel_watchdog', {
        'input_topic': '/cmd_vel',
        'output_topic': '/cmd_vel_safe',
        'timeout_sec': TIMEOUT_SEC,
        'zero_publish_rate_hz': 20.0,
    })

    class Probe(Node):
        def __init__(self):
            super().__init__('watchdog_probe')
            self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.msgs = []
            self.create_subscription(Twist, '/cmd_vel_safe', self.cb, 50)

        def cb(self, msg):
            self.msgs.append((time.time(), msg.linear.x, msg.angular.z))

    probe = Probe()
    start_publish = time.time()
    try:
        end_publish = start_publish + 1.0
        while time.time() < end_publish:
            cmd = Twist()
            cmd.linear.x = 0.3
            cmd.angular.z = 0.1
            probe.pub.publish(cmd)
            rclpy.spin_once(probe, timeout_sec=0.01)
            time.sleep(0.05)

        forwarded = [m for m in probe.msgs if abs(m[1] - 0.3) < 1e-6]
        assert len(forwarded) >= 5, (
            f'commands were not forwarded to /cmd_vel_safe: {probe.msgs}')

        # Keep spinning: the zeros can only be observed if the subscription is
        # serviced while we wait.
        deadline = time.time() + 5.0
        zeros = []
        while time.time() < deadline and not zeros:
            rclpy.spin_once(probe, timeout_sec=0.05)
            zeros = [m for m in probe.msgs
                     if m[0] > end_publish + TIMEOUT_SEC
                     and m[1] == 0.0 and m[2] == 0.0]

        assert zeros, (
            'no zero Twist was published after the command stream went stale')

        assert probe.msgs[-1][1] == 0.0, (
            f'last command sent to the robot is not a stop: {probe.msgs[-1]}')
    finally:
        probe.destroy_node()
