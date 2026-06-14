#!/usr/bin/env python3
"""Drive cyl_1..cyl_4 perpendicular to the L-corridor, gated by services.

Same paths/period/rate as move_corridor_cylinders.py, but idle on startup:
the cylinders sit at their t=0 (phase-0) positions and the timer is a
no-op until /obstacles/start is called.

Services:
  /obstacles/start (std_srvs/Trigger) — record start_time = now() and
      begin moving cylinders, using (now - start_time) as t.
  /obstacles/stop  (std_srvs/Trigger) — stop motion and re-park all
      cylinders at their t=0 positions.

Uses self.get_clock().now(), so respects use_sim_time:=true if passed.

  cyl_1 (red)    — horizontal leg, x=-8, y in [-1.5, 1.5]
  cyl_2 (blue)   — horizontal leg, x=-5, y in [-1.5, 1.5], phase +pi
  cyl_3 (green)  — horizontal leg, x=-2, y in [-1.5, 1.5]
  cyl_4 (yellow) — vertical   leg, y= 3, x in [-0.5, 2.5]
"""
import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from std_srvs.srv import Trigger


PERIOD = 30.0   # seconds per full cycle, all cylinders
TICK   = 1.0 / 200.0   # 200 Hz update
Z      = 0.25   # cylinder centre height (length 0.5 -> bottom on ground)


def shuttle(angle, lo, hi, phase=0.0):
    """Smooth lo -> hi -> lo sweep; angle=0 sits at lo, angle=pi at hi."""
    mid = 0.5 * (lo + hi)
    amp = 0.5 * (hi - lo)
    return mid - amp * math.cos(angle + phase)


class DynamicObstacles(Node):
    def __init__(self):
        super().__init__('dynamic_obstacles')

        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /set_entity_state ...')

        self.start_time = None  # None = idle
        self.create_service(Trigger, '/obstacles/start', self._on_start)
        self.create_service(Trigger, '/obstacles/stop', self._on_stop)
        self.create_timer(TICK, self.tick)

        # Park at t=0 positions on startup.
        self._publish(angle=0.0)
        self.get_logger().info(
            'dynamic_obstacles idle; call /obstacles/start to begin.')

    def _on_start(self, request, response):
        self.start_time = self.get_clock().now()
        self.get_logger().info('Obstacles started.')
        response.success = True
        response.message = 'started'
        return response

    def _on_stop(self, request, response):
        self.start_time = None
        self._publish(angle=0.0)  # re-park
        self.get_logger().info('Obstacles stopped and re-parked.')
        response.success = True
        response.message = 'stopped'
        return response

    def tick(self):
        if self.start_time is None:
            return
        dt_ns = (self.get_clock().now() - self.start_time).nanoseconds
        t = dt_ns / 1e9
        phase = (t % PERIOD) / PERIOD
        angle = 2.0 * math.pi * phase
        self._publish(angle)

    def _publish(self, angle):
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
    node = DynamicObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
