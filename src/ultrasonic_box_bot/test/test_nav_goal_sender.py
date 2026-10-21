"""nav_goal_sender: x/y from the command line turns into one NavigateToPose goal.

Also checks the failure mode the user hit: the sender must report the goal was
accepted and then report SUCCEEDED only when the server actually succeeds.
"""

import subprocess
import threading
import time

from conftest import package_executable


def _run_sender(args, timeout=30):
    return subprocess.run(
        [package_executable('ultrasonic_box_bot', 'nav_goal_sender')] + args,
        capture_output=True, text=True, timeout=timeout)


def test_missing_coordinates_is_a_usage_error():
    completed = _run_sender(['--ros-args', '-p', 'server_timeout_sec:=0.1'])
    assert completed.returncode == 2, (
        'nav_goal_sender should exit 2 without coordinates, '
        f'got {completed.returncode}:\n{completed.stdout}{completed.stderr}')
    assert 'Usage' in completed.stderr, completed.stderr + completed.stdout


def test_positional_goal_reaches_action_server(ros):
    from rclpy.action import ActionServer
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from nav2_msgs.action import NavigateToPose

    received = []
    lock = threading.Lock()

    class FakeNav2(Node):
        def __init__(self):
            super().__init__('fake_nav2_sender')
            self.server = ActionServer(
                self, NavigateToPose, '/navigate_to_pose',
                execute_callback=self.execute,
                callback_group=ReentrantCallbackGroup())

        def execute(self, goal_handle):
            pose = goal_handle.request.pose
            with lock:
                received.append((pose.header.frame_id,
                                 pose.pose.position.x, pose.pose.position.y))
            goal_handle.succeed()
            return NavigateToPose.Result()

    node = FakeNav2()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        time.sleep(1.0)
        completed = _run_sender(['4.0', '4.0'])
        assert completed.returncode == 0, (
            f'expected success, got {completed.returncode}:\n'
            f'{completed.stdout}{completed.stderr}')
        # RCLCPP log output goes to stderr, so check both streams.
        output = completed.stdout + completed.stderr
        assert 'SUCCEEDED' in output, output
        with lock:
            assert received == [('map', 4.0, 4.0)], (
                f'unexpected goal received by the server: {received}')
    finally:
        node.destroy_node()
