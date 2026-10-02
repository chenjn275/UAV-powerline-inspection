#!/usr/bin/env python3
"""Feed the inspection reference path to the real SUPER ROS 2 planner.

The inspection manager owns target discovery, standoff-radius selection and
the coverage path.  SUPER owns the local obstacle-aware trajectory between
successive waypoints.  This node deliberately sends one ``PoseStamped`` goal
at a time so that SUPER's normal replan/FSM path is exercised instead of
turning the reference path into another position controller.

The adapter only advances a goal after the vehicle has reached it according
to the latest ENU odometry.  A fresh ``PositionCommand`` from SUPER is also
required, which makes a missing planner or stale command visible to the PX4
bridge rather than silently completing the mission.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
try:
    from mars_quadrotor_msgs.msg import PositionCommand
except ImportError:  # pragma: no cover - reported during node startup
    PositionCommand = None
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

try:  # PX4 is optional for the standalone SUPER contract test.
    from px4_msgs.msg import VehicleLocalPosition
except ImportError:  # pragma: no cover - exercised without PX4 messages
    VehicleLocalPosition = None


@dataclass(frozen=True)
class _Goal:
    point: tuple[float, float, float]
    yaw: float


class SuperPlannerAdapter(Node):
    """Sequence manager waypoints through SUPER and report mission state."""

    def __init__(self) -> None:
        super().__init__("super_planner_adapter")
        if PositionCommand is None:
            raise RuntimeError(
                "super_planner_adapter requires mars_quadrotor_msgs/msg/PositionCommand; "
                "source the built super_ws overlay first"
            )
        self.declare_parameter("reference_path_topic", "/inspection/reference_path")
        self.declare_parameter("goal_topic", "/inspection/super/goal")
        self.declare_parameter("position_command_topic", "/planning/pos_cmd")
        self.declare_parameter("super_status_topic", "/inspection/super/status")
        self.declare_parameter("position_topic", "/odom")
        self.declare_parameter("px4_position_topic", "/fmu/out/vehicle_local_position")
        self.declare_parameter("px4_position_enu_topic", "/px4_offboard_bridge/position_enu")
        self.declare_parameter("reference_frame", "odom")
        self.declare_parameter("super_frame_id", "world")
        self.declare_parameter("goal_stride", 8)
        self.declare_parameter("goal_tolerance_m", 0.6)
        self.declare_parameter("goal_dwell_s", 0.5)
        self.declare_parameter("command_timeout_s", 1.0)
        self.declare_parameter("position_timeout_s", 1.0)
        self.declare_parameter("position_frame_tolerance_m", 2.0)
        self.declare_parameter("goal_republish_s", 1.0)
        self.declare_parameter(
            "lock_reference_path", True,
        )
        self.declare_parameter("alignment_yaw_rad", 0.0)
        self.declare_parameter("origin_ned", [0.0, 0.0, 0.0])

        self._reference_frame = str(self.get_parameter("reference_frame").value).strip()
        self._super_frame_id = str(self.get_parameter("super_frame_id").value).strip() or "world"
        self._goal_stride = max(1, int(self.get_parameter("goal_stride").value))
        self._goal_tolerance = max(0.05, float(self.get_parameter("goal_tolerance_m").value))
        self._goal_dwell_s = max(0.0, float(self.get_parameter("goal_dwell_s").value))
        self._command_timeout_s = max(0.1, float(self.get_parameter("command_timeout_s").value))
        self._position_timeout_s = max(0.1, float(self.get_parameter("position_timeout_s").value))
        self._position_frame_tolerance = max(
            0.1, float(self.get_parameter("position_frame_tolerance_m").value)
        )
        self._goal_republish_s = max(0.2, float(self.get_parameter("goal_republish_s").value))
        self._lock_reference_path = bool(self.get_parameter("lock_reference_path").value)
        self._alignment_yaw = float(self.get_parameter("alignment_yaw_rad").value)
        origin = tuple(float(value) for value in self.get_parameter("origin_ned").value)
        if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
            raise ValueError("origin_ned must contain three finite values")
        if not math.isfinite(self._alignment_yaw):
            raise ValueError("alignment_yaw_rad must be finite")
        self._origin_ned = origin

        status_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_topic").value), 10
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("super_status_topic").value), status_qos
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("reference_path_topic").value),
            self._on_reference_path,
            10,
        )
        self.create_subscription(
            PositionCommand,
            str(self.get_parameter("position_command_topic").value),
            self._on_position_command,
            sensor_qos,
        )
        self.create_subscription(
            String,
            "/px4_offboard_bridge/status",
            self._on_bridge_status,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("position_topic").value),
            self._on_odom,
            sensor_qos,
        )
        self._px4_sub = None
        if VehicleLocalPosition is not None:
            self._px4_sub = self.create_subscription(
                VehicleLocalPosition,
                str(self.get_parameter("px4_position_topic").value),
                self._on_px4_position,
                sensor_qos,
            )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("px4_position_enu_topic").value),
            self._on_bridge_position,
            sensor_qos,
        )

        self._goals: list[_Goal] = []
        self._path_signature_value: Optional[tuple] = None
        self._path_received_at = 0.0
        self._goal_index = -1
        self._goal_sent_at = 0.0
        self._goal_reached_since: Optional[float] = None
        self._mission_complete = False
        self._path_locked = False
        self._position_enu: Optional[tuple[float, float, float]] = None
        self._position_received_at = 0.0
        self._px4_position_enu: Optional[tuple[float, float, float]] = None
        self._px4_received_at = 0.0
        self._bridge_position_enu: Optional[tuple[float, float, float]] = None
        self._bridge_position_received_at = 0.0
        self._latest_command: Optional[_Goal] = None
        self._command_received_at = 0.0
        self._last_bridge_status = ""
        self._last_status = ""
        self._last_status_at = 0.0

        self.create_timer(0.05, self._tick)
        self._publish_status("SUPER_WAITING_FOR_REFERENCE_PATH")

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        norm = math.sqrt(
            float(q.x) ** 2 + float(q.y) ** 2 + float(q.z) ** 2 + float(q.w) ** 2
        )
        if norm <= 1e-9 or not math.isfinite(norm):
            return 0.0
        return math.atan2(
            2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
            1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2),
        )

    @staticmethod
    def _ned_to_enu(
        point_ned: tuple[float, float, float], origin_ned: tuple[float, float, float], yaw: float
    ) -> tuple[float, float, float]:
        """Invert the bridge's ENU-to-NED local-frame transform."""
        north = point_ned[0] - origin_ned[0]
        east = point_ned[1] - origin_ned[1]
        down = point_ned[2] - origin_ned[2]
        c = math.cos(yaw)
        s = math.sin(yaw)
        # _enu_to_ned applies [[cos, -sin], [sin, cos]] to
        # (north, east).  Use its transpose here so an aligned PX4 pose is
        # compared in exactly the same ENU frame as the reference path.
        return (-s * north + c * east, c * north + s * east, -down)

    @staticmethod
    def _path_signature(goals: list[_Goal]) -> tuple:
        return tuple(
            (round(goal.point[0], 3), round(goal.point[1], 3), round(goal.point[2], 3), round(goal.yaw, 3))
            for goal in goals
        )

    def _on_reference_path(self, msg: Path) -> None:
        frame = str(getattr(msg.header, "frame_id", "")).strip()
        if self._reference_frame and frame != self._reference_frame:
            self._publish_status(f"SUPER_REJECTED_PATH_FRAME frame={frame or '<empty>'}")
            return
        if not msg.poses:
            self._goals = []
            self._path_signature_value = None
            self._goal_index = -1
            self._mission_complete = False
            self._path_locked = False
            self._goal_reached_since = None
            self._publish_status("SUPER_PATH_REVOKED")
            return

        raw_goals: list[_Goal] = []
        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position
            point = (float(p.x), float(p.y), float(p.z))
            if not all(math.isfinite(value) for value in point):
                self._publish_status("SUPER_REJECTED_NONFINITE_PATH")
                return
            yaw = self._yaw_from_quaternion(pose_stamped.pose.orientation)
            raw_goals.append(_Goal(point, yaw))
        if len(raw_goals) < 2:
            self._publish_status("SUPER_REJECTED_PATH_TOO_SHORT")
            return

        indices = list(range(0, len(raw_goals), self._goal_stride))
        if indices[-1] != len(raw_goals) - 1:
            indices.append(len(raw_goals) - 1)
        goals = [raw_goals[index] for index in indices]
        signature = self._path_signature(goals)
        self._path_received_at = time.monotonic()
        if signature == self._path_signature_value:
            return
        # The manager republishes its current path at 5 Hz.  A live FAST-LIO
        # estimate can move by a few centimetres between frames, which would
        # otherwise reset SUPER to goal 1 on every republish and prevent the
        # vehicle from ever advancing around the tower.  A new mission must
        # first revoke the old path (an empty Path); while a path is locked,
        # keep the active sequence and only refresh its watchdog timestamp.
        if self._path_locked and self._lock_reference_path:
            self._publish_status("SUPER_PATH_UPDATE_IGNORED_ACTIVE")
            return
        self._path_signature_value = signature
        self._goals = goals
        self._goal_index = 0
        self._goal_reached_since = None
        self._mission_complete = False
        self._path_locked = True
        self._publish_status(f"SUPER_PATH_READY goals={len(goals)} points={len(raw_goals)}")
        self._send_current_goal()

    def _send_current_goal(self) -> None:
        if not (0 <= self._goal_index < len(self._goals)):
            return
        # Do not hand SUPER a goal before the PX4/FAST-LIO frame has a valid
        # pose.  Sending the click goal first makes the C++ FSM enter
        # GENERATE_TRAJ with no odometry and produces a burst of misleading
        # PlanFromRest failures during normal startup.
        if self._current_position() is None:
            self._goal_sent_at = time.monotonic()
            self._publish_status("SUPER_WAITING_FOR_POSITION")
            return
        goal = self._goals[self._goal_index]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._super_frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = goal.point
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(goal.yaw / 2.0)
        msg.pose.orientation.w = math.cos(goal.yaw / 2.0)
        self._goal_pub.publish(msg)
        self._goal_sent_at = time.monotonic()
        self._goal_reached_since = None
        self._publish_status(f"SUPER_GOAL_SENT index={self._goal_index + 1}/{len(self._goals)}")

    def _on_position_command(self, msg: PositionCommand) -> None:
        p = msg.position
        values = (float(p.x), float(p.y), float(p.z), float(msg.yaw))
        if not all(math.isfinite(value) for value in values):
            return
        self._latest_command = _Goal(values[:3], values[3])
        self._command_received_at = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        values = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(value) for value in values):
            return
        self._position_enu = values
        self._position_received_at = time.monotonic()

    def _on_px4_position(self, msg) -> None:
        values = (
            float(getattr(msg, "x", math.nan)),
            float(getattr(msg, "y", math.nan)),
            float(getattr(msg, "z", math.nan)),
        )
        if not all(math.isfinite(value) for value in values):
            return
        if hasattr(msg, "xy_valid") and not bool(msg.xy_valid):
            return
        if hasattr(msg, "z_valid") and not bool(msg.z_valid):
            return
        self._px4_position_enu = self._ned_to_enu(values, self._origin_ned, self._alignment_yaw)
        self._px4_received_at = time.monotonic()

    def _on_bridge_position(self, msg: PoseStamped) -> None:
        """Consume the bridge's authoritative PX4 position in ENU."""
        frame = str(getattr(getattr(msg, "header", None), "frame_id", "")).strip()
        if frame and self._reference_frame and frame != self._reference_frame:
            return
        p = msg.pose.position
        values = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(value) for value in values):
            return
        self._bridge_position_enu = values
        self._bridge_position_received_at = time.monotonic()

    def _on_bridge_status(self, msg: String) -> None:
        self._last_bridge_status = str(msg.data)

    def _current_position(self) -> Optional[tuple[float, float, float]]:
        now = time.monotonic()
        odom_fresh = (
            self._position_enu is not None
            and now - self._position_received_at <= self._position_timeout_s
        )
        px4_fresh = (
            self._px4_position_enu is not None
            and now - self._px4_received_at <= self._position_timeout_s
        )
        bridge_fresh = (
            self._bridge_position_enu is not None
            and now - self._bridge_position_received_at <= self._position_timeout_s
        )
        if bridge_fresh:
            assert self._bridge_position_enu is not None
            return self._bridge_position_enu
        if odom_fresh and px4_fresh:
            assert self._position_enu is not None
            assert self._px4_position_enu is not None
            frame_error = math.dist(self._position_enu, self._px4_position_enu)
            if frame_error > self._position_frame_tolerance:
                # FAST-LIO may start in camera_init while PX4 starts in its
                # local NED origin.  Do not let that frame-reset error leave
                # the aircraft waiting forever at one SUPER goal.  The PX4
                # local position is the same position used by the bridge's
                # setpoint and is therefore the safe progress source until
                # the two estimates are aligned.
                self._publish_status(
                    "SUPER_POSITION_FRAME_MISMATCH "
                    f"error={frame_error:.2f}m source=PX4_LOCAL"
                )
                return self._px4_position_enu
            return self._position_enu
        if odom_fresh:
            return self._position_enu
        if px4_fresh:
            return self._px4_position_enu
        return None

    def _tick(self) -> None:
        if self._mission_complete:
            self._publish_status("SUPER_MISSION_COMPLETE")
            return
        if not self._goals or self._goal_index < 0:
            self._publish_status("SUPER_WAITING_FOR_REFERENCE_PATH")
            return
        now = time.monotonic()
        if now - self._path_received_at > max(2.0, 3.0 * self._command_timeout_s):
            self._publish_status("SUPER_STALE_REFERENCE_PATH")
            return
        if self._latest_command is None or now - self._command_received_at > self._command_timeout_s:
            # The first goal can be published before the C++ FSM has finished
            # subscribing.  Keep retrying the same goal until SUPER produces
            # a fresh PositionCommand; this is also the recovery path after a
            # transient planner restart.
            if now - self._goal_sent_at >= self._goal_republish_s:
                self._send_current_goal()
            self._publish_status("SUPER_WAITING_FOR_COMMAND")
            return
        position = self._current_position()
        if position is None:
            self._publish_status("SUPER_WAITING_FOR_POSITION")
            return
        goal = self._goals[self._goal_index]
        distance = math.dist(position, goal.point)
        if distance > self._goal_tolerance:
            self._goal_reached_since = None
            return
        if self._goal_reached_since is None:
            self._goal_reached_since = now
            self._publish_status(
                f"SUPER_GOAL_REACHED_PENDING index={self._goal_index + 1}/{len(self._goals)} distance={distance:.2f}"
            )
            return
        if now - self._goal_reached_since < self._goal_dwell_s:
            return
        self._publish_status(
            f"SUPER_GOAL_REACHED index={self._goal_index + 1}/{len(self._goals)} distance={distance:.2f}"
        )
        if self._goal_index == len(self._goals) - 1:
            self._mission_complete = True
            self._publish_status("SUPER_MISSION_COMPLETE")
            return
        self._goal_index += 1
        self._send_current_goal()

    def _publish_status(self, text: str) -> None:
        now = time.monotonic()
        if text == self._last_status and now - self._last_status_at < 0.75:
            return
        self._last_status = text
        self._last_status_at = now
        self._status_pub.publish(String(data=text))
        self.get_logger().info(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = SuperPlannerAdapter()
    except (ImportError, ValueError) as exc:
        print(f"super_planner_adapter cannot start: {exc}")
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
