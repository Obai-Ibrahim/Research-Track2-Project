#!/usr/bin/env python3
"""Run hard-coded Nav2 trials in sequence with obstacle sync + metric logging.

Per trial:
  1) /obstacles/stop          (park cylinders at t=0)
  2) teleport robot to start
  3) BasicNavigator.setInitialPose(start)   <-- IMMEDIATELY after teleport
  4) BasicNavigator.clearAllCostmaps()      <-- twice, with 0.5 s gap
  5) sleep 5 s          (robot held still before any motion command)
  6) /obstacles/start  +  BasicNavigator.goToPose(goal)
  7) poll ground-truth state at 10 Hz, logging
         (t, x, y, yaw, v, min_clearance)
     until nav.isTaskComplete() OR 90 s sim-time timeout
     (on timeout, nav.cancelTask() is called)
  8) /obstacles/stop
  9) compute end-of-trial metrics and append one row to the CSV.
 (+) sleep 1 s before the next trial (skipped after the last)

Metrics per row:
  run_id, trial_idx,
  start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw,
  success, timeout,
  time_to_goal, path_length, path_efficiency, straight_line,
  min_clearance, mean_jerk, n_close_approach

Run with --ros-args -p use_sim_time:=true so /clock and self.get_clock()
agree. CLI:
    --output  PATH    (default: results/run.csv)
    --run-id  STR     (default: timestamp like '20260615_180000')
"""
import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from std_srvs.srv import Trigger

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# (start_pose, goal_pose) — each pose = (x, y, yaw_radians)
TRIALS = [
    ((1.0, 5.0, 0.0), (0.0, 0.0, 0.0)),
    ((-2.0, -1.0, 1.57),  (2.0, 1.5, 0.0)),
    ((0.0, 0.0, 0.0), (-4.0, -1.0, 0.0)),
    ((0.0, 0.0, 0.0), (-4.0, -1.0, 0.0)),
    ((1.0, 5.0, 0.0), (0.0, 0.0, 0.0))
]

ENTITY_NAMES = ['waffle', 'turtlebot3_waffle']
TELEPORT_SERVICES = ['/gazebo/set_entity_state', '/set_entity_state']

# Adjust to match how your world names dynamic cylinders. Any model whose
# name starts with one of these prefixes is treated as a moving cylinder.
CYLINDER_NAME_PREFIXES = ('moving_cylinder_', 'cyl_')
# gazebo_ros publishes /model_states in the default namespace; if your
# launch namespaces it under /gazebo we also subscribe to that.
MODEL_STATES_TOPICS = ('/model_states', '/gazebo/model_states')

SAMPLE_PERIOD            = 0.1     # seconds — 10 Hz logging
TIMEOUT_SECONDS          = 90.0    # sim-time per-trial budget
CLOSE_APPROACH_THRESHOLD = 0.30    # m — counts as "close approach"

CSV_COLUMNS = [
    'run_id', 'trial_idx',
    'start_x', 'start_y', 'start_yaw',
    'goal_x',  'goal_y',  'goal_yaw',
    'success', 'timeout',
    'time_to_goal', 'path_length', 'path_efficiency',
    'straight_line', 'min_clearance', 'mean_jerk', 'n_close_approach',
]


def yaw_to_quat_zw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def make_pose_stamped(x, y, yaw, stamp, frame_id='map'):
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = stamp
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    qz, qw = yaw_to_quat_zw(yaw)
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p


class SimpleDriver(Node):
    def __init__(self):
        super().__init__('simple_driver')

        self.start_cli = self.create_client(Trigger, '/obstacles/start')
        self.stop_cli  = self.create_client(Trigger, '/obstacles/stop')
        for cli, name in [(self.start_cli, '/obstacles/start'),
                          (self.stop_cli,  '/obstacles/stop')]:
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'Waiting for {name} ...')

        self.gz_cli = None
        for srv in TELEPORT_SERVICES:
            cli = self.create_client(SetEntityState, srv)
            if cli.wait_for_service(timeout_sec=2.0):
                self.gz_cli = cli
                self.get_logger().info(f'Using {srv} for teleport.')
                break
        if self.gz_cli is None:
            raise RuntimeError(
                f'None of {TELEPORT_SERVICES} are available.')

        # Ground-truth state stream from Gazebo. Subscribe to every candidate
        # topic; whichever is actually publishing will fill latest_states.
        self.latest_states = None
        for topic in MODEL_STATES_TOPICS:
            self.create_subscription(
                ModelStates, topic, self._on_model_states, 10)

        # Sanity check — warn early if neither candidate is publishing.
        wait_deadline = time.monotonic() + 5.0
        while self.latest_states is None and time.monotonic() < wait_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_states is None:
            self.get_logger().warn(
                f'No ModelStates received on any of {MODEL_STATES_TOPICS} '
                'within 5 s — metrics will be empty. '
                'Check `ros2 topic list | grep model_states` while sim is up.')
        else:
            self.get_logger().info(
                'ModelStates flowing — first message has '
                f'{len(self.latest_states.name)} entities.')

    def _on_model_states(self, msg):
        self.latest_states = msg

    def call_trigger(self, cli):
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def teleport(self, x, y, yaw):
        qz, qw = yaw_to_quat_zw(yaw)
        for entity_name in ENTITY_NAMES:
            req = SetEntityState.Request()
            req.state.name = entity_name
            req.state.pose.position.x = float(x)
            req.state.pose.position.y = float(y)
            req.state.pose.position.z = 0.0
            req.state.pose.orientation.z = qz
            req.state.pose.orientation.w = qw
            req.state.reference_frame = 'world'
            fut = self.gz_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            res = fut.result()
            if res is not None and res.success:
                return True
            self.get_logger().info(
                f"Teleport entity '{entity_name}' failed; trying next.")
        return False

    def _sample_from_states(self, t_elapsed):
        """Return (t, x, y, yaw, v, min_clearance) from latest_states, or None."""
        msg = self.latest_states
        if msg is None:
            return None

        robot_idx = None
        for name in ENTITY_NAMES:
            if name in msg.name:
                robot_idx = msg.name.index(name)
                break
        if robot_idx is None:
            return None

        pose  = msg.pose[robot_idx]
        twist = msg.twist[robot_idx]
        x = pose.position.x
        y = pose.position.y
        yaw = quat_to_yaw(pose.orientation)
        v = math.hypot(twist.linear.x, twist.linear.y)

        cyl_xys = [
            (msg.pose[i].position.x, msg.pose[i].position.y)
            for i, n in enumerate(msg.name)
            if any(n.startswith(p) for p in CYLINDER_NAME_PREFIXES)
        ]
        if cyl_xys:
            min_clearance = min(math.hypot(x - cx, y - cy) for cx, cy in cyl_xys)
        else:
            min_clearance = float('inf')

        return (t_elapsed, x, y, yaw, v, min_clearance)

    def collect_trial(self, nav):
        """Poll ground-truth at 10 Hz until nav completes or timeout fires.

        Returns (log, timeout_fired).
        """
        trial_start = nav.get_clock().now()
        log = []
        next_sample_at = 0.0
        timeout_fired = False

        while True:
            # Process ModelStates callback so latest_states is fresh.
            rclpy.spin_once(self, timeout_sec=0.0)

            now = nav.get_clock().now()
            elapsed = (now - trial_start).nanoseconds / 1e9

            # isTaskComplete spins nav for up to 100 ms, giving the loop ~10 Hz.
            if nav.isTaskComplete():
                break

            if elapsed > TIMEOUT_SECONDS:
                self.get_logger().warn(
                    f'Trial timeout ({TIMEOUT_SECONDS:.0f} s) — cancelling.')
                nav.cancelTask()
                timeout_fired = True
                while not nav.isTaskComplete():
                    rclpy.spin_once(self, timeout_sec=0.0)
                break

            if elapsed >= next_sample_at:
                sample = self._sample_from_states(elapsed)
                if sample is not None:
                    log.append(sample)
                next_sample_at += SAMPLE_PERIOD

        return log, timeout_fired


RESULT_LABEL = {
    TaskResult.SUCCEEDED: 'SUCCEEDED',
    TaskResult.CANCELED:  'CANCELED',
    TaskResult.FAILED:    'FAILED',
}


def compute_metrics(log, start_xy, goal_xy):
    """Reduce the per-step log to a dict of summary metrics."""
    straight_line = math.hypot(goal_xy[0] - start_xy[0],
                               goal_xy[1] - start_xy[1])

    if len(log) == 0:
        return {
            'time_to_goal': 0.0,
            'path_length': 0.0,
            'path_efficiency': 0.0,
            'straight_line': straight_line,
            'min_clearance': float('inf'),
            'mean_jerk': 0.0,
            'n_close_approach': 0,
        }

    t  = np.array([s[0] for s in log], dtype=float)
    xs = np.array([s[1] for s in log], dtype=float)
    ys = np.array([s[2] for s in log], dtype=float)
    v  = np.array([s[4] for s in log], dtype=float)
    cl = [s[5] for s in log]

    time_to_goal = float(t[-1])
    path_length  = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
    path_efficiency = (straight_line / path_length) if path_length > 0 else 0.0

    finite_cl = [c for c in cl if c is not None and not math.isinf(c)]
    min_clearance = float(min(finite_cl)) if finite_cl else float('inf')

    if len(t) < 4:
        mean_jerk = 0.0
    else:
        acc  = np.gradient(v, t)
        jerk = np.gradient(acc, t)
        mean_jerk = float(np.mean(np.abs(jerk)))

    n_close_approach = sum(
        1 for c in cl if c is not None and c < CLOSE_APPROACH_THRESHOLD)

    return {
        'time_to_goal': time_to_goal,
        'path_length': path_length,
        'path_efficiency': path_efficiency,
        'straight_line': straight_line,
        'min_clearance': min_clearance,
        'mean_jerk': mean_jerk,
        'n_close_approach': n_close_approach,
    }


def append_csv_row(path, row):
    """Append one row to the CSV at path. Write header if file is new."""
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    new_file = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        if new_file:
            writer.writeheader()
        writer.writerow(row)


def parse_cli_args():
    parser = argparse.ArgumentParser(allow_abbrev=False, add_help=False)
    parser.add_argument('--output', default='results/run_dwb.csv')
    parser.add_argument(
        '--run-id', default=datetime.now().strftime('%Y%m%d_%H%M%S'))
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_cli_args()
    rclpy.init()

    driver = SimpleDriver()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()

    for i, (start, goal) in enumerate(TRIALS, start=1):
        # 1. park cylinders
        driver.call_trigger(driver.stop_cli)

        # 2. teleport robot to start
        driver.teleport(*start)

        # 3. tell AMCL where we are IMMEDIATELY (no sleep before this)
        init_stamp = nav.get_clock().now().to_msg()
        nav.setInitialPose(make_pose_stamped(*start, stamp=init_stamp))

        # 4. clear costmaps twice with a small gap to wipe transient marks
        nav.clearAllCostmaps()
        time.sleep(0.5)
        nav.clearAllCostmaps()

        # 5. hold still for 5 s before releasing obstacles + goal
        time.sleep(5.0)

        # 6. release the obstacles and send the goal
        driver.call_trigger(driver.start_cli)
        goal_stamp = nav.get_clock().now().to_msg()
        nav.goToPose(make_pose_stamped(*goal, stamp=goal_stamp))

        # 7. collect 10 Hz ground-truth log until completion or timeout
        log, timeout_fired = driver.collect_trial(nav)

        # 8. stop obstacles
        driver.call_trigger(driver.stop_cli)

        # 9. report + compute metrics + CSV row
        result = nav.getResult()
        success = (result == TaskResult.SUCCEEDED) and not timeout_fired
        print(f'Trial {i} done: '
              f'result={RESULT_LABEL.get(result, str(result))}, '
              f'success={success}, timeout={timeout_fired}, '
              f'samples={len(log)}')

        metrics = compute_metrics(log,
                                  (start[0], start[1]),
                                  (goal[0],  goal[1]))
        row = {
            'run_id':   args.run_id,
            'trial_idx': i,
            'start_x':  start[0], 'start_y': start[1], 'start_yaw': start[2],
            'goal_x':   goal[0],  'goal_y':  goal[1],  'goal_yaw':  goal[2],
            'success':  success,
            'timeout':  timeout_fired,
            **metrics,
        }
        append_csv_row(args.output, row)

        # Inter-trial pause (skip after the last trial)
        if i < len(TRIALS):
            time.sleep(1.0)

    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
