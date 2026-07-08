#!/usr/bin/env python3
"""
maze_path_planner_node.py – ROS node for Duckietown maze path planning.

Reads start/goal parameters, runs Dijkstra's algorithm on the maze graph,
and publishes the planned path as a latched String topic.

Published topics
----------------
/maze/path          (std_msgs/String)   JSON-encoded list of node names.
/maze/path_costs    (std_msgs/String)   JSON-encoded list of segment costs.
/maze/maneuvers     (std_msgs/String)   JSON-encoded list of maneuvers.

Parameters
----------
~start_node   (str, default 'A')  Start node name.
~goal_node    (str, default 'E')  Goal node name.
"""

import json
import sys
import os

import rospy
from std_msgs.msg import String

# Allow importing sibling modules whether run via rosrun or directly
_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

from map_definition import get_maneuver, load_maze_from_file  # noqa: E402
from dijkstra import dijkstra                          # noqa: E402

try:
    from duckietown.dtros import DTROS, NodeType
    _USE_DTROS = True
except ImportError:
    _USE_DTROS = False


# ---------------------------------------------------------------------------
# Node implementation
# ---------------------------------------------------------------------------

class MazePathPlannerNode(DTROS if _USE_DTROS else object):
    """Plans and publishes the shortest path through the Duckietown maze."""

    def __init__(self, node_name: str = 'maze_path_planner_node'):
        if _USE_DTROS:
            super().__init__(node_name=node_name, node_type=NodeType.PLANNING)
        else:
            rospy.init_node(node_name)

        # Parameters
        self._start = rospy.get_param('~start_node', 'A')
        self._goal = rospy.get_param('~goal_node', 'E')
        self._map_file = rospy.get_param(
            '~map_file',
            os.environ.get('DUCKIE_MAZE_MAP', '/data/assets/maze_map.yaml'),
        )

        self._node_positions, self._maze_graph = load_maze_from_file(self._map_file)

        # Publishers (latched so late subscribers still receive the path)
        self._pub_path = rospy.Publisher(
            '/maze/path', String, queue_size=1, latch=True)
        self._pub_path_costs = rospy.Publisher(
            '/maze/path_costs', String, queue_size=1, latch=True)
        self._pub_maneuvers = rospy.Publisher(
            '/maze/maneuvers', String, queue_size=1, latch=True)

        rospy.loginfo(f"[PathPlanner] Planning route: {self._start} → {self._goal}")
        rospy.loginfo(f"[PathPlanner] Map source: {self._map_file}")
        self._plan_and_publish()

    # ------------------------------------------------------------------
    def _plan_and_publish(self) -> None:
        path, cost = dijkstra(self._maze_graph, self._start, self._goal)

        if path is None:
            rospy.logerr(
                f"[PathPlanner] No path found from '{self._start}' to '{self._goal}'")
            return

        rospy.loginfo(
            f"[PathPlanner] Shortest path ({cost:.0f} tiles): {' → '.join(path)}")

        segment_costs = []
        for i in range(len(path) - 1):
            segment_costs.append(
                self._get_edge_cost(path[i], path[i + 1])
            )

        # Build maneuver list for every transition
        maneuvers = []
        for i in range(len(path)):
            prev_node = path[i - 1] if i > 0 else path[0]
            curr_node = path[i]
            next_node = path[i + 1] if i < len(path) - 1 else None
            maneuver = get_maneuver(prev_node, curr_node, next_node, self._node_positions)
            maneuvers.append({'node': curr_node, 'maneuver': maneuver})

        self._pub_path.publish(String(data=json.dumps(path)))
        self._pub_path_costs.publish(String(data=json.dumps(segment_costs)))
        self._pub_maneuvers.publish(String(data=json.dumps(maneuvers)))

        rospy.loginfo(f"[PathPlanner] Maneuvers: {maneuvers}")
        rospy.loginfo(f"[PathPlanner] Segment costs: {segment_costs}")

    def _get_edge_cost(self, from_node: str, to_node: str) -> float:
        """Return tile-cost for one path segment, falling back to 1.0."""
        for neighbour, edge_cost in self._maze_graph.get(from_node, []):
            if neighbour == to_node:
                return float(edge_cost)
        return 1.0


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    node = MazePathPlannerNode()
    rospy.spin()
