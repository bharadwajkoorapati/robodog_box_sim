"""Shared fixtures for the node-level tests.

The tests run the installed executables as real processes and talk to them over
DDS, which is the only way to check behaviour that lives in timers and
subscription callbacks.  A per-process ROS_DOMAIN_ID keeps the tests isolated
from each other and from a running simulation.
"""

import os
import signal
import subprocess
import time

import pytest

from ament_index_python.packages import get_package_prefix

# Must be set before rclpy.init() in any test, hence at import time.
os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)


def package_executable(package, executable):
    """Absolute path of an installed executable (avoids the `ros2 run` wrapper)."""
    return os.path.join(
        get_package_prefix(package), 'lib', package, executable)


@pytest.fixture
def ros():
    """rclpy lifecycle for tests that publish/subscribe in-process."""
    import rclpy
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def spawn():
    """Start package executables and make sure they die with the test.

    `ros2 run` is deliberately not used: it leaves the child node alive when the
    wrapper is killed, which makes later tests see stale publishers.
    """
    procs = []

    def _spawn(package, executable, params=None, settle_sec=1.5):
        cmd = [package_executable(package, executable)]
        if params:
            cmd.append('--ros-args')
            for key, value in params.items():
                cmd += ['-p', f'{key}:={value}']
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
        procs.append(proc)
        time.sleep(settle_sec)
        if proc.poll() is not None:
            raise AssertionError(
                f'{executable} exited during start-up:\n{proc.stdout.read()}')
        return proc

    yield _spawn

    for proc in procs:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
    for proc in procs:
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def wait_until(predicate, timeout_sec, message):
    """Poll `predicate` until it returns True or fail with `message`."""
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError(message)
