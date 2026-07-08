#!/usr/bin/env python3
"""
maze_navigator_node.py – ROS node for Duckietown maze navigation.

Combines the planned path, current localisation, and obstacle detection to
decide what the Duckiebot should do at each moment:
  * Follow the lane (controller takes care of this)
  * Execute an intersection maneuver (straight / left / right)
  * Stop when an obstacle is detected
  * Stop and celebrate when the goal is reached

Subscribed topics
-----------------
/maze/maneuvers       (std_msgs/String)  JSON list of {node, maneuver} dicts.
/maze/current_node    (std_msgs/String)  Name of the current maze node.

Published topics
----------------
/maze/nav_command     (std_msgs/String)  Current command:
                                          'go', 'stop', or maneuver with node id
                                          (e.g. 'left@I', 'straight@S').
/maze/goal_reached    (std_msgs/String)  Published once with value 'true' on arrival.

Parameters
----------
~vehicle_name     (str,   default 'duckiebot')
~goal_node        (str,   default 'T')
~intersection_wait_s (float, default 1.5)  Pause before executing turn.
"""

import json
import rospy
from std_msgs.msg import String
import sys
import os

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

from obstacle_detector import ObstacleDetector   # noqa: E402

try:
    from duckietown.dtros import DTROS, NodeType
    _USE_DTROS = True
except ImportError:
    _USE_DTROS = False


class MazeNavigatorNode(DTROS if _USE_DTROS else object):
    """Issues navigation commands based on path, localisation, and obstacles."""

    def __init__(self, node_name: str = 'maze_navigator_node'):
        if _USE_DTROS:
            super().__init__(node_name=node_name, node_type=NodeType.CONTROL)
        else:
            rospy.init_node(node_name)

        # Parameters
        self._vehicle = rospy.get_param('~vehicle_name', 'duckiebot')
        self._goal_node = rospy.get_param('~goal_node', 'T')
        self._intersection_wait = rospy.get_param('~intersection_wait_s', 1.5)

        # State
        self._maneuvers: list = []          # [{node, maneuver}, ...]
        self._current_node: str = ''
        self._active_maneuver_node: str = ''
        self._active_maneuver_cmd: str = ''
        self._active_expected_next: str = ''
        self._goal_reached: bool = False
        self._last_command: str = 'stop'

        # Obstacle detector
        self._obstacle = ObstacleDetector(self._vehicle)

        # Publishers
        self._pub_cmd = rospy.Publisher(
            '/maze/nav_command', String, queue_size=1)
        self._pub_goal = rospy.Publisher(
            '/maze/goal_reached', String, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('/maze/maneuvers', String, self._cb_maneuvers, queue_size=1)
        rospy.Subscriber('/maze/current_node', String, self._cb_current_node, queue_size=1)

        # Control loop at 10 Hz
        rospy.Timer(rospy.Duration(0.1), self._control_loop)

        rospy.loginfo('[Navigator] Node started.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _cb_maneuvers(self, msg: String) -> None:
        self._maneuvers = json.loads(msg.data)
        rospy.loginfo(f'[Navigator] Received {len(self._maneuvers)} maneuvers.')

    def _cb_current_node(self, msg: String) -> None:
        new_node = msg.data
        if new_node != self._current_node:
            rospy.loginfo(f'[Navigator] Now at node: {new_node}')

            if self._active_maneuver_cmd and new_node == self._active_expected_next:
                rospy.loginfo(
                    f'[Navigator] Maneuver {self._active_maneuver_cmd} completed: '
                    f'{self._active_maneuver_node} -> {new_node}')
                self._active_maneuver_node = ''
                self._active_maneuver_cmd = ''
                self._active_expected_next = ''

        self._current_node = new_node

        # Check for goal
        if new_node == self._goal_node and not self._goal_reached:
            self._goal_reached = True
            rospy.loginfo('[Navigator] GOAL REACHED!')
            self._pub_goal.publish(String(data='true'))

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def _control_loop(self, _event) -> None:
        if self._goal_reached:
            self._publish_command('stop')
            return

        if self._obstacle.obstacle_detected:
            self._publish_command('stop')
            return

        if not self._current_node or not self._maneuvers:
            self._publish_command('stop')
            return

        # Find the maneuver for the current node
        maneuver = self._get_maneuver_for(self._current_node)

        if maneuver == 'stop':
            self._publish_command('stop')
        elif maneuver in ('left', 'right', 'straight'):
            expected_next = self._get_next_node_for(self._current_node)

            if (
                self._active_maneuver_node != self._current_node
                or self._active_maneuver_cmd != maneuver
            ):
                self._active_maneuver_node = self._current_node
                self._active_maneuver_cmd = maneuver
                self._active_expected_next = expected_next
                rospy.loginfo(f'[Navigator] Intersection at {self._current_node}: {maneuver}')

            # Keep publishing until the localisation reports that we reached
            # the expected next node on the path.
            self._publish_command(f'{maneuver}@{self._current_node}')
        else:
            self._publish_command('go')

            # If we have moved away from a maneuver node without matching the
            # expected next node, clear stale state and continue safely.
            if self._active_maneuver_cmd and self._current_node != self._active_maneuver_node:
                self._active_maneuver_node = ''
                self._active_maneuver_cmd = ''
                self._active_expected_next = ''

    def _get_maneuver_for(self, node: str) -> str:
        for entry in self._maneuvers:
            if entry.get('node') == node:
                return entry.get('maneuver', 'straight')
        return 'go'

    def _get_next_node_for(self, node: str) -> str:
        for i, entry in enumerate(self._maneuvers):
            if entry.get('node') == node:
                if i + 1 < len(self._maneuvers):
                    return self._maneuvers[i + 1].get('node', '')
                return ''
        return ''

    def _publish_command(self, command: str) -> None:
        self._last_command = command
        self._pub_cmd.publish(String(data=command))


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    node = MazeNavigatorNode()
    rospy.spin()
