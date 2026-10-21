"""map_padder: the padded grid covers the arena and preserves the real map.

This is what stops "Goal Coordinates ... was outside bounds": once the grid
covers the whole arena, a goal in an unexplored region is inside the costmap.
"""

import time


def test_pads_to_arena_and_keeps_cells(ros, spawn):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid

    latch = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)

    spawn('ultrasonic_box_bot', 'map_padder', {
        'input_topic': '/map',
        'output_topic': '/map_padded',
        'min_x': -6.0, 'max_x': 6.0,
        'min_y': -6.0, 'max_y': 6.0,
    })

    class Probe(Node):
        def __init__(self):
            super().__init__('map_padder_probe')
            self.pub = self.create_publisher(OccupancyGrid, '/map', latch)
            self.padded = None
            self.create_subscription(
                OccupancyGrid, '/map_padded', self.cb, latch)

        def cb(self, msg):
            self.padded = msg

    probe = Probe()

    # Small raw map, like SLAM early in a run: 2 x 2 m at 0.1 m, origin (1, 1)
    # so it sits in the +x/+y quadrant and leaves (4, 4) outside it.
    raw = OccupancyGrid()
    raw.header.frame_id = 'map'
    raw.info.resolution = 0.1
    raw.info.width = 20
    raw.info.height = 20
    raw.info.origin.position.x = 1.0
    raw.info.origin.position.y = 1.0
    raw.info.origin.orientation.w = 1.0
    raw.data = [0] * (20 * 20)
    raw.data[0] = 100  # occupied cell at the map's (1.0, 1.0) corner

    try:
        # Spin while publishing: discovery of the padder's subscriber can take a
        # moment, and the transient-local latch only covers publishes made after
        # the subscription exists.
        deadline = time.time() + 10.0
        while time.time() < deadline and probe.padded is None:
            probe.pub.publish(raw)
            rclpy.spin_once(probe, timeout_sec=0.05)
        assert probe.padded is not None, 'map_padder did not publish /map_padded'

        padded = probe.padded
        min_x = padded.info.origin.position.x
        min_y = padded.info.origin.position.y
        max_x = min_x + padded.info.width * padded.info.resolution
        max_y = min_y + padded.info.height * padded.info.resolution

        assert 4.0 < max_x and 4.0 < max_y, (
            f'padded grid does not cover the goal (4, 4): '
            f'x up to {max_x:.2f}, y up to {max_y:.2f}')
        assert min_x <= -5.0 and min_y <= -5.0, (
            f'padded grid does not cover the negative arena: origin {min_x}, {min_y}')

        # The original cell must be preserved at the same map coordinates.
        col = round((1.0 - min_x) / padded.info.resolution)
        row = round((1.0 - min_y) / padded.info.resolution)
        index = row * padded.info.width + col
        assert padded.data[index] == 100, (
            f'real map data was not copied through (expected 100, got '
            f'{padded.data[index]})')

        # Cells outside the raw map must be unknown, not free.
        far = (round((4.0 - min_y) / padded.info.resolution) * padded.info.width
               + round((4.0 - min_x) / padded.info.resolution))
        assert padded.data[far] == -1, (
            f'unexplored arena cells should be unknown (-1), got {padded.data[far]}')
    finally:
        probe.destroy_node()
