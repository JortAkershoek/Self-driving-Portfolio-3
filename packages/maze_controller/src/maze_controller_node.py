#!/usr/bin/env python3
"""
maze_controller_node.py – PID lane-keeping and intersection controller.

Architecture
------------
The controller combines two control loops:

1. **Lane-keeping PID** (runs continuously while driving)
   - Error = lateral offset from lane centre (from LanePose)
   - Output = steering correction (differential wheel speed)

2. **Navigation command handler**
   - Reads /maze/nav_command and adjusts behaviour:
     * 'go'       → normal lane-following at cruise speed
     * 'stop'     → zero wheel commands
    * 'straight@<node>' → straight-through intersection at node
    * 'left@<node>'     → left-turn maneuver at node
    * 'right@<node>'    → right-turn maneuver at node

Subscribed topics
-----------------
/<veh>/lane_filter_node/lane_pose     (duckietown_msgs/LanePose)
/maze/nav_command                     (std_msgs/String)

Published topics
----------------
/<veh>/wheels_driver_node/wheels_cmd  (duckietown_msgs/WheelsCmdStamped)

Parameters
----------
~vehicle_name      (str,   default 'duckiebot')
~v_cruise          (float, default 0.20)  Cruise speed (m/s)
~v_turn            (float, default 0.15)  Speed during turns
~pid_kp            (float, default 5.0)   PID proportional gain
~pid_ki            (float, default 0.05)  PID integral gain
~pid_kd            (float, default 0.5)   PID derivative gain
~turn_duration_s   (float, default 1.8)   Duration of a left/right turn
~straight_duration_s (float, default 1.0) Duration of open-loop straight crossing
~lane_timeout_s    (float, default 0.25)  Max age of lane data before stopping
"""

import rospy
import sys
import os
from std_msgs.msg import String

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

from pid_controller import PIDController   # noqa: E402

try:
    from duckietown_msgs.msg import WheelsCmdStamped, LanePose
    _HAS_DT_MSGS = True
except ImportError:
    _HAS_DT_MSGS = False
    rospy.logwarn('[Controller] duckietown_msgs not found – wheel commands disabled.')

try:
    from duckietown.dtros import DTROS, NodeType
    _USE_DTROS = True
except ImportError:
    _USE_DTROS = False


# Wheel speed limits (Duckiebot hardware constraint)
MAX_WHEEL_SPEED = 1.0   # normalised units


class MazeControllerNode(DTROS if _USE_DTROS else object):
    """PID lane-following controller with intersection maneuver support."""

    def __init__(self, node_name: str = 'maze_controller_node'):
        if _USE_DTROS:
            super().__init__(node_name=node_name, node_type=NodeType.CONTROL)
        else:
            rospy.init_node(node_name)

        # Parameters
        self._vehicle = rospy.get_param('~vehicle_name', 'duckiebot')
        self._v_cruise = rospy.get_param('~v_cruise', 0.20)
        self._v_turn = rospy.get_param('~v_turn', 0.15)
        self._turn_duration = rospy.get_param('~turn_duration_s', 1.8)
        self._straight_duration = rospy.get_param('~straight_duration_s', 1.0)
        self._lane_timeout = rospy.get_param('~lane_timeout_s', 0.25)

        # PID setup
        kp = rospy.get_param('~pid_kp', 5.0)
        ki = rospy.get_param('~pid_ki', 0.05)
        kd = rospy.get_param('~pid_kd', 0.5)
        self._pid = PIDController(kp=kp, ki=ki, kd=kd,
                                  output_limits=(-MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))

        # State
        self._nav_command: str = 'stop'
        self._lane_error: float = 0.0   # lateral offset (metres, + = too far right)
        self._turning: bool = False
        self._turn_end_time: float = 0.0
        self._turn_direction: str = 'straight'
        self._last_executed_node = None
        self._last_lane_pose_time: float = 0.0
        self._last_pid_time: float = 0.0
        self._last_lane_cmd = (0.0, 0.0)

        # Publisher
        if _HAS_DT_MSGS:
            self._pub_wheels = rospy.Publisher(
                f'/{self._vehicle}/wheels_driver_node/wheels_cmd',
                WheelsCmdStamped,
                queue_size=1,
            )

        # Subscribers
        if _HAS_DT_MSGS:
            rospy.Subscriber(
                f'/{self._vehicle}/lane_filter_node/lane_pose',
                LanePose,
                self._cb_lane_pose,
                queue_size=1,
            )
        rospy.Subscriber('/maze/nav_command', String, self._cb_nav_command, queue_size=1)

        # Control loop at 20 Hz
        rospy.Timer(rospy.Duration(0.05), self._control_loop)

        rospy.loginfo('[Controller] Node started.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _cb_lane_pose(self, msg) -> None:
        """LanePose: d = lateral offset (m), phi = heading error (rad)."""
        # Combine lateral offset and heading error into a single error signal.
        # Weighting: lateral offset dominates, heading adds stability.
        self._lane_error = msg.d + 0.15 * msg.phi
        now = rospy.get_time()
        self._last_lane_pose_time = now

        # Only compute steering when lane following is active.
        if self._turning or self._nav_command not in ('go', 'straight'):
            return

        if self._last_pid_time <= 0.0:
            dt = 0.05
        else:
            dt = now - self._last_pid_time
            if dt <= 0.0:
                return

        self._last_pid_time = now
        correction = self._pid.compute(self._lane_error, dt)
        self._last_lane_cmd = self._compute_lane_command(correction)

    def _cb_nav_command(self, msg: String) -> None:
        new_cmd, node_id = self._parse_nav_command(msg.data)
        if new_cmd == 'stop':
            self._turning = False
            self._nav_command = 'stop'
            self._pid.reset()
            self._last_pid_time = 0.0
            return

        if new_cmd == 'go':
            # Unlock maneuver latch when navigator confirms progression.
            self._last_executed_node = None
            self._nav_command = 'go'
            return

        execution_key = node_id if node_id else f'cmd:{new_cmd}'

        if (
            new_cmd in ('left', 'right', 'straight')
            and not self._turning
            and execution_key != self._last_executed_node
        ):
            # Start timed open-loop intersection maneuver
            self._turning = True
            self._turn_direction = new_cmd
            self._last_executed_node = execution_key
            duration = self._straight_duration if new_cmd == 'straight' else self._turn_duration
            self._turn_end_time = rospy.get_time() + duration
            self._pid.reset()
            self._last_pid_time = 0.0
            if node_id:
                rospy.loginfo(f'[Controller] Starting {new_cmd} maneuver at node {node_id}.')
            else:
                rospy.loginfo(f'[Controller] Starting {new_cmd} maneuver.')
            return

        if self._turning:
            return

        if new_cmd not in ('left', 'right', 'straight'):
            self._nav_command = new_cmd

    @staticmethod
    def _parse_nav_command(raw_cmd: str):
        """
        Parse nav command payload.

        Supports both legacy format ('left') and node-tagged format
        ('left@I'). Returns (command, node_id_or_empty_string).
        """
        text = (raw_cmd or '').strip()
        if '@' not in text:
            return text, ''

        cmd, node_id = text.split('@', 1)
        return cmd.strip(), node_id.strip()

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def _control_loop(self, _event) -> None:
        now = rospy.get_time()

        # Finish timed intersection turn
        if self._turning:
            if now >= self._turn_end_time:
                self._turning = False
                self._nav_command = 'go'
                self._pid.reset()
                self._last_pid_time = 0.0
                # Prime a neutral forward command so we never reuse stale
                # pre-maneuver steering values on the first cycle after turning.
                self._last_lane_cmd = (self._v_cruise, self._v_cruise)
                self._last_lane_pose_time = now
                self._send_wheel_cmd(*self._last_lane_cmd)
                rospy.loginfo('[Controller] Maneuver complete – resuming lane following.')
                return
            else:
                self._execute_turn(self._turn_direction)
                return

        # Normal command dispatch
        if self._nav_command == 'stop':
            self._send_wheel_cmd(0.0, 0.0)
        elif self._nav_command in ('go', 'straight'):
            lane_age = now - self._last_lane_pose_time if self._last_lane_pose_time > 0.0 else float('inf')
            if lane_age <= self._lane_timeout:
                self._send_wheel_cmd(*self._last_lane_cmd)
            else:
                self._send_wheel_cmd(0.0, 0.0)
                rospy.logwarn_throttle(
                    1.0,
                    '[Controller] Lane pose timeout; stopping until fresh data arrives.')
        else:
            self._send_wheel_cmd(0.0, 0.0)

    # ------------------------------------------------------------------
    # Lane following
    # ------------------------------------------------------------------
    def _compute_lane_command(self, correction: float):
        """Convert PID correction into clamped left/right wheel speeds."""
        # Differential drive: steer by adjusting left/right wheel speeds.
        # Positive error = too far right → increase left, decrease right.
        v_left = self._v_cruise + correction
        v_right = self._v_cruise - correction

        # Normalise to MAX_WHEEL_SPEED
        v_left = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, v_left))
        v_right = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, v_right))

        return v_left, v_right

    # ------------------------------------------------------------------
    # Intersection turns
    # ------------------------------------------------------------------
    def _execute_turn(self, direction: str) -> None:
        """
        Execute a fixed-speed differential turn.

        Left turn:  slow left wheel, fast right wheel.
        Right turn: fast left wheel, slow right wheel.
        """
        v = self._v_turn
        v_slow = min(MAX_WHEEL_SPEED, v * 0.35)
        v_fast = min(MAX_WHEEL_SPEED, v * 1.2)
        if direction == 'left':
            self._send_wheel_cmd(v_slow, v_fast)
        elif direction == 'right':
            self._send_wheel_cmd(v_fast, v_slow)
        else:
            self._send_wheel_cmd(v, v)

    # ------------------------------------------------------------------
    def _send_wheel_cmd(self, v_left: float, v_right: float) -> None:
        if not _HAS_DT_MSGS:
            return
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = v_left
        msg.vel_right = v_right
        self._pub_wheels.publish(msg)


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    node = MazeControllerNode()
    rospy.spin()
