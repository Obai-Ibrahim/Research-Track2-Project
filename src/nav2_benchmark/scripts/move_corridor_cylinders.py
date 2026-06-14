#!/usr/bin/env python3
"""Drive cyl_1..cyl_4 perpendicular to the L-corridor via /set_entity_state.

Period 30 s, 200 Hz ticks. Shuttles use phase-shifted cosine so velocity
goes to zero at endpoints — smooth motion, no jerk into the costmap.

Max instantaneous speed ~0.31 m/s (mid-stroke); step ~1.6 mm per tick.
Phase is computed from a monotonic wall clock so a missed timer fire
doesn't drift or cause a visible jump.

  cyl_1 (red)    — horizontal leg, x=-8, y in [-1.5, 1.5]
  cyl_2 (blue)   — horizontal leg, x=-5, y in [-1.5, 1.5], phase +pi
  cyl_3 (green)  — horizontal leg, x=-2, y in [-1.5, 1.5]
  cyl_4 (yellow) — vertical   leg, y= 3, x in [-0.5, 2.5]
"""
import math
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState


PERIOD = 30.0   # seconds per full cycle, all cylinders
TICK   = 1.0 / 200.0   # 200 Hz update
Z      = 0.25   # cylinder centre height (length 0.5 -> bottom on ground)


def shuttle(angle, lo, hi, phase=0.0):
    """Smooth lo -> hi -> lo sweep; angle=0 sits at lo, angle=pi at hi."""
    mid = 0.5 * (lo + hi)
    amp = 0.5 * (hi - lo)
    return mid - amp * math.cos(angle + phase)


class MoveCylinders(Node):
    def __init__(self):
        super().__init__('move_corridor_cylinders')
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /set_entity_state ...')

        self.t0 = time.monotonic()
        self.create_timer(TICK, self.tick)
        self.get_logger().info('move_corridor_cylinders running at 200 Hz, 30 s sweeps.')

    def tick(self):
        t = time.monotonic() - self.t0
        phase = (t % PERIOD) / PERIOD
        angle = 2.0 * math.pi * phase

        # Horizontal-leg cylinders: shuttle in y (perpendicular to corridor)
        self._set('cyl_1', -8.0, shuttle(angle, -1.5, 1.5))
        self._set('cyl_2', -5.0, shuttle(angle, -1.5, 1.5, math.pi))
        self._set('cyl_3', -2.0, shuttle(angle, -1.5, 1.5))

        # Vertical-leg cylinder: shuttle in x (perpendicular to corridor)
        self._set('cyl_4', shuttle(angle, -0.5, 2.5), 3.0)

    def _set(self, name, x, y):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = Z
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        fut = self.cli.call_async(req)
        fut.add_done_callback(lambda _f: None)


def main():
    rclpy.init()
    node = MoveCylinders()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
