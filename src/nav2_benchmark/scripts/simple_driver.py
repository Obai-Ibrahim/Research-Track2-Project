#!/usr/bin/env python3
"""Run two hard-coded Nav2 trials in sequence, syncing with dynamic_obstacles.

Smoke test of the start/stop sync. Assumes Nav2 is already up (e.g. via
benchmark.launch.py). For each trial:
  1) /obstacles/stop          (park cylinders at t=0)
  2) teleport robot to start
  3) BasicNavigator.setInitialPose(start)   <-- IMMEDIATELY after teleport
  4) BasicNavigator.clearAllCostmaps()      <-- wipe any transient garbage
  5) sleep 7 s          (robot held still before any motion command)
  6) /obstacles/start  +  BasicNavigator.goToPose(goal)
  7) wait until nav.isTaskComplete()
  8) print "Trial X done: result=<...>"
  9) /obstacles/stop
 (+) sleep 7 s before the next trial (skipped after the last)

ORDER MATTERS: do not sleep between teleport and setInitialPose. While
Gazebo's body is at the new pose but AMCL's TF still says the old pose,
incoming /scan rays get projected into the global costmap via the WRONG
map->base_link transform. That polluted costmap is what you see as a
"crash". Calling setInitialPose immediately narrows that window to a
few ms, and the subsequent clearAllCostmaps wipes whatever sneaked in.

Edit TRIALS below to match your map. Run with
    --ros-args -p use_sim_time:=true
"""
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
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


def yaw_to_quat_zw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


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

        # Probe both common teleport service names and use whichever is up.
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


RESULT_LABEL = {
    TaskResult.SUCCEEDED: 'SUCCEEDED',
    TaskResult.CANCELED:  'CANCELED',
    TaskResult.FAILED:    'FAILED',
}


def main():
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

        # 4. clear costmaps to wipe any obstacle marks written during the
        #    brief TF-mismatch window between teleport and setInitialPose.
        #    Two passes with a small gap: the first clears whatever is in
        #    the costmap; the gap lets in-flight scans (under the now-correct
        #    TF) land; the second pass catches any obstacle layer pollution
        #    that arrived between AMCL accepting /initialpose and the first
        #    clear taking effect.
        nav.clearAllCostmaps()
        time.sleep(0.5)
        nav.clearAllCostmaps()

        # 5. hold still for 5 s before releasing obstacles + goal
        time.sleep(5.0)

        # 6. release the obstacles and send the goal
        driver.call_trigger(driver.start_cli)
        goal_stamp = nav.get_clock().now().to_msg()
        nav.goToPose(make_pose_stamped(*goal, stamp=goal_stamp))

        # 7. wait for nav to finish
        while not nav.isTaskComplete():
            pass

        # 8. report
        result = nav.getResult()
        print(f'Trial {i} done: result={RESULT_LABEL.get(result, str(result))}')

        # 9. park cylinders again
        driver.call_trigger(driver.stop_cli)

        # Inter-trial pause (skip after the last trial)
        if i < len(TRIALS):
            time.sleep(1.0)

    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
