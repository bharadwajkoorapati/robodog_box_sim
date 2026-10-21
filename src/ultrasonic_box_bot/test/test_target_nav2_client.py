"""target_nav2_client: one NavigateToPose goal per target, in the map frame.

A 10 Hz target publisher must not turn into 10 goals per second, and the goal
must be transformed out of the target's frame (`odom`) into the navigation
frame (`map`).  A fake action server stands in for Nav2.
"""

import subprocess
import threading
import time

from conftest import package_executable, wait_until


def test_one_goal_per_target_in_map_frame(ros, spawn):
    from rclpy.action import ActionServer
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose

    goals = []
    lock = threading.Lock()

    class FakeNav2(Node):
        def __init__(self):
            super().__init__('fake_nav2')
            self.server = ActionServer(
                self, NavigateToPose, '/navigate_to_pose',
                execute_callback=self.execute,
                callback_group=ReentrantCallbackGroup())

        def execute(self, goal_handle):
            pose = goal_handle.request.pose
            with lock:
                goals.append((pose.header.frame_id,
                              pose.pose.position.x, pose.pose.position.y))
            time.sleep(0.2)
            goal_handle.succeed()
            return NavigateToPose.Result()

    class TargetPublisher(Node):
        def __init__(self):
            super().__init__('fake_target_pub')
            self.pub = self.create_publisher(
                PoseStamped, '/target_cube/pose', 10)
            self.x = 4.0
            self.y = -4.0

        def send(self):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.orientation.w = 1.0
            self.pub.publish(msg)

    # Identity map -> odom stands in for SLAM/AMCL.
    tf_proc = subprocess.Popen(
        [package_executable('tf2_ros', 'static_transform_publisher'),
         '0', '0', '0', '0', '0', '0', 'map', 'odom'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)

    target = TargetPublisher()
    nav = FakeNav2()
    executor = MultiThreadedExecutor()
    executor.add_node(target)
    executor.add_node(nav)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        spawn('ultrasonic_box_bot', 'target_nav2_client', {
            'start_delay_sec': 1.0,
            'retry_period_sec': 0.2,
            'tf_timeout_sec': 1.0,
        })

        def hold(x, y, seconds):
            target.x, target.y = x, y
            end = time.time() + seconds
            while time.time() < end:
                target.send()
                time.sleep(0.1)

        hold(4.0, -4.0, 4.0)
        with lock:
            first = list(goals)
        hold(2.0, 2.0, 3.0)
        with lock:
            second = list(goals)

        assert len(first) == 1, (
            f'expected exactly 1 goal for a constant target, got {first}')
        assert first[0][0] == 'map', (
            f'goal frame is {first[0][0]!r}, expected map')
        assert abs(first[0][1] - 4.0) < 1e-3 and abs(first[0][2] + 4.0) < 1e-3, (
            f'goal coordinates wrong: {first[0]}')

        wait_until(lambda: len(second) == 2, 3.0,
                   f'expected a second goal after the target moved, got {second}')
        assert abs(second[1][1] - 2.0) < 1e-3 and abs(second[1][2] - 2.0) < 1e-3, (
            f'second goal coordinates wrong: {second[1]}')
    finally:
        tf_proc.terminate()
        tf_proc.wait(timeout=5)
        nav.destroy_node()
        target.destroy_node()
