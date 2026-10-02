#!/usr/bin/env python3
"""Guarded inspection-path adapter for PX4 Offboard.

The node is deliberately disabled by default.  When enabled, it accepts a
ROS ENU ``nav_msgs/Path``, converts it to PX4 local NED, and publishes the
PX4 heartbeat/setpoint messages only while all of these gates are true:

* a fresh path with the configured frame id is available;
* fresh PX4 local position, VehicleStatus and TimesyncStatus messages are available; and
* ``enable_setpoints`` was explicitly set to true.

Offboard mode, arming, and landing requests are separately disabled by
default.  This bridge is for the explicitly configured SITL integration path;
it is not a physical-flight approval.
"""

from __future__ import annotations

import math
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


# PX4 publishes the versioned VehicleStatus uORB message on this ROS 2 topic.
PX4_VEHICLE_STATUS_TOPIC = "/fmu/out/vehicle_status_v1"


class Px4OffboardBridge(Node):
    def __init__(self) -> None:
        super().__init__("px4_offboard_bridge")
        self.declare_parameter("path_topic", "/inspection/reference_path")
        self.declare_parameter("vehicle_position_topic", "/fmu/out/vehicle_local_position")
        self.declare_parameter("vehicle_status_topic", PX4_VEHICLE_STATUS_TOPIC)
        self.declare_parameter("timesync_topic", "/fmu/out/timesync_status")
        self.declare_parameter("status_topic", "/px4_offboard_bridge/status")
        self.declare_parameter("position_enu_topic", "/px4_offboard_bridge/position_enu")
        self.declare_parameter("expected_frame", "map")
        self.declare_parameter("fmu_prefix", "/fmu")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("path_timeout_s", 0.5)
        self.declare_parameter("timesync_timeout_s", 1.5)
        self.declare_parameter("vehicle_status_timeout_s", 1.5)
        self.declare_parameter("path_duration_s", 75.0)
        self.declare_parameter("tracking_error_pause_m", 2.0)
        self.declare_parameter("reference_speed_limit_m_s", 1.5)
        self.declare_parameter("reference_yaw_rate_limit_rad_s", 0.5)
        self.declare_parameter("enable_setpoints", False)
        self.declare_parameter("request_offboard", False)
        self.declare_parameter("request_arm", False)
        self.declare_parameter("land_on_path_end", False)
        self.declare_parameter("use_super_position_commands", False)
        self.declare_parameter("super_command_topic", "/planning/pos_cmd")
        self.declare_parameter("super_status_topic", "/inspection/super/status")
        self.declare_parameter("super_command_timeout_s", 1.0)
        self.declare_parameter("land_on_super_mission_complete", True)
        self.declare_parameter("path_end_tolerance_m", 0.75)
        self.declare_parameter("command_retry_s", 2.0)
        self.declare_parameter("max_command_attempts", 5)
        self.declare_parameter("alignment_yaw_rad", 0.0)
        self.declare_parameter("origin_ned", [0.0, 0.0, 0.0])

        self._path_topic = str(self.get_parameter("path_topic").value)
        self._position_topic = str(self.get_parameter("vehicle_position_topic").value)
        self._vehicle_status_topic = str(self.get_parameter("vehicle_status_topic").value)
        self._timesync_topic = str(self.get_parameter("timesync_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._position_enu_topic = str(self.get_parameter("position_enu_topic").value)
        self._expected_frame = str(self.get_parameter("expected_frame").value)
        self._fmu_prefix = str(self.get_parameter("fmu_prefix").value).rstrip("/") or "/fmu"
        self._rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self._path_timeout_s = max(0.05, float(self.get_parameter("path_timeout_s").value))
        self._timesync_timeout_s = max(0.1, float(self.get_parameter("timesync_timeout_s").value))
        self._vehicle_status_timeout_s = max(0.1, float(self.get_parameter("vehicle_status_timeout_s").value))
        self._path_duration_s = max(1.0, float(self.get_parameter("path_duration_s").value))
        self._tracking_error_pause_m = max(0.25, float(self.get_parameter("tracking_error_pause_m").value))
        self._reference_speed_limit_m_s = max(0.1, float(self.get_parameter("reference_speed_limit_m_s").value))
        self._reference_yaw_rate_limit_rad_s = max(
            0.05, float(self.get_parameter("reference_yaw_rate_limit_rad_s").value)
        )
        self._enable_setpoints = bool(self.get_parameter("enable_setpoints").value)
        self._request_offboard = bool(self.get_parameter("request_offboard").value)
        self._request_arm = bool(self.get_parameter("request_arm").value)
        self._land_on_path_end = bool(self.get_parameter("land_on_path_end").value)
        self._use_super_position_commands = bool(self.get_parameter("use_super_position_commands").value)
        self._super_command_topic = str(self.get_parameter("super_command_topic").value)
        self._super_status_topic = str(self.get_parameter("super_status_topic").value)
        self._super_command_timeout_s = max(
            0.1, float(self.get_parameter("super_command_timeout_s").value)
        )
        self._land_on_super_mission_complete = bool(
            self.get_parameter("land_on_super_mission_complete").value
        )
        self._path_end_tolerance_m = max(0.1, float(self.get_parameter("path_end_tolerance_m").value))
        self._command_retry_s = max(0.25, float(self.get_parameter("command_retry_s").value))
        self._max_command_attempts = max(1, int(self.get_parameter("max_command_attempts").value))
        self._alignment_yaw = float(self.get_parameter("alignment_yaw_rad").value)
        origin = tuple(float(value) for value in self.get_parameter("origin_ned").value)
        if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
            raise ValueError("origin_ned must contain three finite values")
        if not math.isfinite(self._alignment_yaw):
            raise ValueError("alignment_yaw_rad must be finite")
        self._origin_ned = origin

        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        # Publish the same PX4 local position used for setpoint tracking after
        # converting it into the bridge's configured ENU frame.  The SUPER
        # goal sequencer consumes this explicit boundary topic instead of
        # guessing whether FAST-LIO's odom origin has been aligned with PX4.
        self._position_enu_pub = self.create_publisher(PoseStamped, self._position_enu_topic, 10)
        self._path_points_ned: Optional[list[tuple[float, float, float]]] = None
        self._path_yaws_ned: Optional[list[float]] = None
        self._mission_started_at: Optional[float] = None
        self._mission_elapsed_s = 0.0
        self._mission_last_tick_at: Optional[float] = None
        self._path_received_at = 0.0
        self._last_rejected_frame = ""
        self._super_command_enu: Optional[tuple[float, float, float]] = None
        self._super_command_yaw_enu: Optional[float] = None
        self._super_command_received_at = 0.0
        self._super_status = ""
        self._super_mission_complete = False
        self._super_path_ready = False
        self._super_path_revoked = False
        self._vehicle_position: Optional[tuple[float, float, float]] = None
        self._vehicle_received_at = 0.0
        self._vehicle_status_received_at = 0.0
        self._vehicle_armed: Optional[bool] = None
        self._vehicle_nav_state: Optional[int] = None
        self._last_target_ned: Optional[tuple[float, float, float]] = None
        self._last_target_yaw: Optional[float] = None
        self._last_target_at: Optional[float] = None
        self._timesync_offset_us: Optional[int] = None
        self._timesync_received_at = 0.0
        self._warmup_ticks = 0
        self._mode_requested = False
        self._arm_requested = False
        self._mode_request_attempts = 0
        self._arm_request_attempts = 0
        self._last_mode_request_at = 0.0
        self._last_arm_request_at = 0.0
        self._first_mode_request_at = 0.0
        self._land_request_attempts = 0
        self._last_land_request_at = 0.0
        self._land_requested = False
        self._landing_active = False
        self._mission_complete = False
        self._setpoint_count = 0
        self._last_status = ""
        self._last_status_at = 0.0

        self.create_subscription(Path, self._path_topic, self._on_path, 10)
        self._super_types: Optional[dict[str, Any]] = None
        if self._use_super_position_commands:
            self._load_super_types()
            assert self._super_types is not None
            super_qos = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                self._super_types["PositionCommand"],
                self._super_command_topic,
                self._on_super_command,
                super_qos,
            )
            self.create_subscription(String, self._super_status_topic, self._on_super_status, 10)
        self._px4_types: Optional[dict[str, Any]] = None
        self._px4_publishers: dict[str, Any] = {}
        if self._enable_setpoints:
            self._load_px4_types()

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        if self._enable_setpoints:
            assert self._px4_types is not None
            types = self._px4_types
            self._vehicle_sub = self.create_subscription(
                types["VehicleLocalPosition"], self._position_topic, self._on_vehicle_position, qos
            )
            self._timesync_sub = self.create_subscription(
                types["TimesyncStatus"], self._timesync_topic, self._on_timesync, qos
            )
            self._vehicle_status_sub = self.create_subscription(
                types["VehicleStatus"], self._vehicle_status_topic, self._on_vehicle_status, qos
            )
            self._px4_publishers["offboard"] = self.create_publisher(
                types["OffboardControlMode"], f"{self._fmu_prefix}/in/offboard_control_mode", qos
            )
            self._px4_publishers["setpoint"] = self.create_publisher(
                types["TrajectorySetpoint"], f"{self._fmu_prefix}/in/trajectory_setpoint", qos
            )
            self._px4_publishers["command"] = self.create_publisher(
                types["VehicleCommand"], f"{self._fmu_prefix}/in/vehicle_command", qos
            )
        else:
            self.get_logger().warning(
                "setpoint output is disabled; this node is in dry-run mode and will not publish /fmu/in messages"
            )

        self.create_timer(1.0 / self._rate_hz, self._tick)
        self._publish_status("DISABLED_DRY_RUN" if not self._enable_setpoints else "WAITING_FOR_PATH")

    def _load_px4_types(self) -> None:
        try:
            from px4_msgs.msg import (  # type: ignore
                OffboardControlMode,
                TimesyncStatus,
                TrajectorySetpoint,
                VehicleCommand,
                VehicleLocalPosition,
                VehicleStatus,
            )
        except ImportError:
            # Some older generated Python packages leave msg/__init__.py
            # empty.  Accept their generated modules when they are complete,
            # while still failing clearly if a partial build is installed.
            try:
                from px4_msgs.msg._offboard_control_mode import OffboardControlMode  # type: ignore
                from px4_msgs.msg._timesync_status import TimesyncStatus  # type: ignore
                from px4_msgs.msg._trajectory_setpoint import TrajectorySetpoint  # type: ignore
                from px4_msgs.msg._vehicle_command import VehicleCommand  # type: ignore
                from px4_msgs.msg._vehicle_local_position import VehicleLocalPosition  # type: ignore
                from px4_msgs.msg._vehicle_status import VehicleStatus  # type: ignore
            except (ImportError, AttributeError) as module_exc:
                raise RuntimeError(
                    "enable_setpoints=true requires a complete px4_msgs package from the "
                    "PX4-matched ROS workspace; the current Humble interface generator "
                    "cannot build this host's px4_msgs package"
                ) from module_exc
        self._px4_types = {
            "OffboardControlMode": OffboardControlMode,
            "TimesyncStatus": TimesyncStatus,
            "TrajectorySetpoint": TrajectorySetpoint,
            "VehicleCommand": VehicleCommand,
            "VehicleLocalPosition": VehicleLocalPosition,
            "VehicleStatus": VehicleStatus,
        }

    def _load_super_types(self) -> None:
        try:
            from mars_quadrotor_msgs.msg import PositionCommand  # type: ignore
        except (ImportError, AttributeError) as exc:
            raise RuntimeError(
                "use_super_position_commands=true requires the installed "
                "mars_quadrotor_msgs/msg/PositionCommand interface from super_ws"
            ) from exc
        self._super_types = {"PositionCommand": PositionCommand}

    @staticmethod
    def _enu_to_ned(
        position_enu: tuple[float, float, float], origin_ned: tuple[float, float, float], yaw: float
    ) -> tuple[float, float, float]:
        east, north, up = position_enu
        c = math.cos(yaw)
        s = math.sin(yaw)
        return (
            origin_ned[0] + c * north - s * east,
            origin_ned[1] + s * north + c * east,
            origin_ned[2] - up,
        )

    @staticmethod
    def _ned_to_enu(
        position_ned: tuple[float, float, float], origin_ned: tuple[float, float, float], yaw: float
    ) -> tuple[float, float, float]:
        """Convert PX4 local NED feedback into the bridge's ENU frame."""
        north = float(position_ned[0]) - float(origin_ned[0])
        east = float(position_ned[1]) - float(origin_ned[1])
        down = float(position_ned[2]) - float(origin_ned[2])
        c = math.cos(float(yaw))
        s = math.sin(float(yaw))
        return (-s * north + c * east, c * north + s * east, -down)

    @staticmethod
    def _enu_yaw_to_ned(yaw_enu: float, alignment_yaw: float = 0.0) -> float:
        """Convert ROS ENU yaw (CCW from east) to PX4 NED yaw (CW from north)."""
        yaw_ned = math.pi / 2.0 - float(yaw_enu) + float(alignment_yaw)
        return math.atan2(math.sin(yaw_ned), math.cos(yaw_ned))

    @staticmethod
    def _sample_path(
        points: list[tuple[float, float, float]],
        yaws: list[float],
        progress: float,
    ) -> tuple[tuple[float, float, float], float, int]:
        """Interpolate a time-parameterized path without jumping between turns."""
        if len(points) != len(yaws) or not points:
            raise ValueError("path points and yaws must be non-empty and have matching lengths")
        u = min(1.0, max(0.0, float(progress))) * (len(points) - 1)
        index = min(len(points) - 1, int(math.floor(u)))
        next_index = min(len(points) - 1, index + 1)
        alpha = u - index
        point = tuple(
            points[index][axis] + alpha * (points[next_index][axis] - points[index][axis])
            for axis in range(3)
        )
        yaw_delta = math.atan2(
            math.sin(yaws[next_index] - yaws[index]),
            math.cos(yaws[next_index] - yaws[index]),
        )
        yaw = math.atan2(
            math.sin(yaws[index] + alpha * yaw_delta),
            math.cos(yaws[index] + alpha * yaw_delta),
        )
        return point, yaw, index

    def _on_path(self, msg: Path) -> None:
        frame = msg.header.frame_id.strip()
        if frame != self._expected_frame:
            if frame != self._last_rejected_frame:
                self.get_logger().error(
                    f"rejecting path frame '{frame or '<empty>'}'; expected '{self._expected_frame}'"
                )
                self._last_rejected_frame = frame
            self._path_points_ned = None
            self._path_yaws_ned = None
            self._publish_status("REJECTED_PATH_FRAME")
            return
        points: list[tuple[float, float, float]] = []
        yaws: list[float] = []
        for pose_stamped in msg.poses:
            position = pose_stamped.pose.position
            enu_point = (float(position.x), float(position.y), float(position.z))
            if not all(math.isfinite(value) for value in enu_point):
                self._path_points_ned = None
                self._path_yaws_ned = None
                self._publish_status("REJECTED_NONFINITE_PATH")
                return
            points.append(self._enu_to_ned(enu_point, self._origin_ned, self._alignment_yaw))
            orientation = pose_stamped.pose.orientation
            q_norm = math.sqrt(
                float(orientation.x) ** 2 + float(orientation.y) ** 2
                + float(orientation.z) ** 2 + float(orientation.w) ** 2
            )
            yaw_enu = (
                2.0 * math.atan2(float(orientation.z) / q_norm, float(orientation.w) / q_norm)
                if q_norm > 1e-9 else 0.0
            )
            yaws.append(self._enu_yaw_to_ned(yaw_enu, self._alignment_yaw))
        if len(points) < 2:
            self._path_points_ned = None
            self._path_yaws_ned = None
            self._publish_status("REJECTED_PATH_TOO_SHORT")
            return
        first_valid_path = self._path_points_ned is None
        self._path_points_ned = points
        self._path_yaws_ned = yaws
        self._path_received_at = time.monotonic()
        self._last_rejected_frame = ""
        # The manager republishes the same fresh path at 5 Hz. Resetting the
        # warmup on each copy made the 10-tick Offboard gate unreachable.
        if first_valid_path:
            self._warmup_ticks = 0
        self._publish_status(f"PATH_READY points={len(points)}")

    def _on_super_command(self, msg: Any) -> None:
        """Accept a fresh SUPER ENU position command for PX4 conversion."""
        position = getattr(msg, "position", None)
        if position is None:
            return
        point = (float(position.x), float(position.y), float(position.z))
        yaw = float(getattr(msg, "yaw", math.nan))
        if not all(math.isfinite(value) for value in (*point, yaw)):
            self._publish_status("REJECTED_NONFINITE_SUPER_COMMAND")
            return
        self._super_command_enu = point
        self._super_command_yaw_enu = yaw
        self._super_command_received_at = time.monotonic()
        self._super_path_ready = True
        self._super_path_revoked = False

    def _on_super_status(self, msg: String) -> None:
        status = str(msg.data)
        self._super_status = status
        if status.startswith("SUPER_PATH_READY"):
            self._super_path_ready = True
            self._super_path_revoked = False
            self._super_mission_complete = False
        elif status in {"SUPER_PATH_REVOKED", "SUPER_STALE_REFERENCE_PATH"}:
            self._super_path_ready = False
            self._super_path_revoked = True
            self._super_mission_complete = False
        elif status == "SUPER_MISSION_COMPLETE":
            self._super_mission_complete = True

    def _on_vehicle_position(self, msg: Any) -> None:
        x = float(getattr(msg, "x", math.nan))
        y = float(getattr(msg, "y", math.nan))
        z = float(getattr(msg, "z", math.nan))
        if not all(math.isfinite(value) for value in (x, y, z)):
            return
        if hasattr(msg, "xy_valid") and not bool(msg.xy_valid):
            return
        if hasattr(msg, "z_valid") and not bool(msg.z_valid):
            return
        self._vehicle_position = (x, y, z)
        self._vehicle_received_at = time.monotonic()
        east, north, up = self._ned_to_enu((x, y, z), self._origin_ned, self._alignment_yaw)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._expected_frame
        pose.pose.position.x = float(east)
        pose.pose.position.y = float(north)
        pose.pose.position.z = float(up)
        pose.pose.orientation.w = 1.0
        self._position_enu_pub.publish(pose)

    def _on_vehicle_status(self, msg: Any) -> None:
        assert self._px4_types is not None
        status_type = self._px4_types["VehicleStatus"]
        armed_state = int(getattr(status_type, "ARMING_STATE_ARMED", 2))
        self._vehicle_armed = int(getattr(msg, "arming_state", 0)) == armed_state
        self._vehicle_nav_state = int(getattr(msg, "nav_state", -1))
        self._vehicle_status_received_at = time.monotonic()

    def _on_timesync(self, msg: Any) -> None:
        offset = int(getattr(msg, "estimated_offset", 0))
        self._timesync_offset_us = offset
        self._timesync_received_at = time.monotonic()

    def _px4_timestamp(self) -> Optional[int]:
        if self._timesync_offset_us is None:
            return None
        if time.monotonic() - self._timesync_received_at > self._timesync_timeout_s:
            return None
        ros_us = self.get_clock().now().nanoseconds // 1000
        return max(0, int(ros_us - self._timesync_offset_us))

    @staticmethod
    def _ease_progress(progress: float) -> float:
        """Apply a smooth start/stop profile to normalized path progress."""
        value = min(1.0, max(0.0, float(progress)))
        return value * value * (3.0 - 2.0 * value)

    @staticmethod
    def _limit_position_step(
        previous: tuple[float, float, float],
        target: tuple[float, float, float],
        dt: float,
        max_speed: float,
    ) -> tuple[float, float, float]:
        """Bound setpoint jumps when a replanned path shifts its geometry."""
        delta = tuple(target[axis] - previous[axis] for axis in range(3))
        distance = math.sqrt(sum(value * value for value in delta))
        max_step = max(0.0, float(dt)) * max(0.0, float(max_speed))
        if distance <= max_step or distance <= 1e-12:
            return target
        scale = max_step / distance
        return tuple(previous[axis] + scale * delta[axis] for axis in range(3))

    @staticmethod
    def _limit_yaw_step(previous: float, target: float, dt: float, max_rate: float) -> float:
        delta = math.atan2(math.sin(target - previous), math.cos(target - previous))
        step = max(-max(0.0, dt) * max(0.0, max_rate), min(max(0.0, dt) * max(0.0, max_rate), delta))
        result = previous + step
        return math.atan2(math.sin(result), math.cos(result))

    def _publish_heartbeat(self, timestamp: int) -> None:
        assert self._px4_types is not None
        msg = self._px4_types["OffboardControlMode"]()
        msg.timestamp = timestamp
        for field in ("position", "velocity", "acceleration", "attitude", "body_rate", "thrust_and_torque", "direct_actuator"):
            if hasattr(msg, field):
                setattr(msg, field, field == "position")
        self._px4_publishers["offboard"].publish(msg)

    def _publish_setpoint(self, timestamp: int, point: tuple[float, float, float], yaw: float) -> None:
        assert self._px4_types is not None
        msg = self._px4_types["TrajectorySetpoint"]()
        msg.timestamp = timestamp
        msg.position = [float(point[0]), float(point[1]), float(point[2])]
        nan = float("nan")
        if hasattr(msg, "velocity"):
            msg.velocity = [nan, nan, nan]
        if hasattr(msg, "acceleration"):
            msg.acceleration = [nan, nan, nan]
        if hasattr(msg, "jerk"):
            msg.jerk = [nan, nan, nan]
        if hasattr(msg, "yaw"):
            msg.yaw = float(yaw)
        if hasattr(msg, "yawspeed"):
            msg.yawspeed = nan
        self._px4_publishers["setpoint"].publish(msg)

    def _publish_command(self, timestamp: int, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        assert self._px4_types is not None
        msg = self._px4_types["VehicleCommand"]()
        msg.timestamp = timestamp
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._px4_publishers["command"].publish(msg)

    def _tick(self) -> None:
        if not self._enable_setpoints:
            return
        now = time.monotonic()
        status_type = self._px4_types["VehicleStatus"]
        offboard_state = int(getattr(status_type, "NAVIGATION_STATE_OFFBOARD", 14))
        auto_land_state = int(getattr(status_type, "NAVIGATION_STATE_AUTO_LAND", 18))
        nav_state = self._vehicle_nav_state
        status_fresh = now - self._vehicle_status_received_at <= self._vehicle_status_timeout_s

        if status_fresh and nav_state == auto_land_state and self._land_requested:
            self._landing_active = True
        if self._mission_complete:
            self._publish_status("PX4_DISARMED_AFTER_LAND")
            return
        if self._landing_active:
            if status_fresh and self._vehicle_armed is False:
                self._mission_complete = True
                self._publish_status("PX4_DISARMED_AFTER_LAND")
            elif status_fresh:
                self._publish_status("PX4_LANDING_ACTIVE")
            else:
                self._publish_status("WAITING_FOR_PX4_STATUS_AFTER_LAND")
            return
        if status_fresh and nav_state == auto_land_state:
            self._publish_status("PX4_UNEXPECTED_AUTO_LAND; setpoints stopped")
            return
        if self._mission_started_at is not None:
            if not status_fresh:
                self._publish_status("STALE_PX4_STATUS_OUTPUT_INHIBITED")
                return
            if nav_state != offboard_state:
                self._publish_status(f"PX4_MODE_EXITED nav_state={nav_state}; setpoints stopped")
                return

        if self._use_super_position_commands:
            self._tick_super(now, status_fresh, nav_state, offboard_state)
            return

        if self._path_points_ned is None:
            self._publish_status("WAITING_FOR_PATH")
            return
        if now - self._path_received_at > self._path_timeout_s:
            self._publish_status("STALE_PATH_OUTPUT_INHIBITED")
            return
        if self._vehicle_position is None or now - self._vehicle_received_at > self._timesync_timeout_s:
            self._publish_status("WAITING_FOR_VALID_PX4_POSITION")
            return
        timestamp = self._px4_timestamp()
        if timestamp is None:
            self._publish_status("WAITING_FOR_TIMESYNC")
            return

        self._publish_heartbeat(timestamp)
        in_offboard_flight = self._vehicle_armed is True and nav_state == offboard_state
        if in_offboard_flight:
            if self._mission_started_at is None:
                self._mission_started_at = now
                self._mission_last_tick_at = now
            assert self._path_yaws_ned is not None
            progress = min(1.0, self._mission_elapsed_s / self._path_duration_s)
            hold_point, _, _ = self._sample_path(
                self._path_points_ned, self._path_yaws_ned, self._ease_progress(progress)
            )
            tracking_error = math.dist(self._vehicle_position, hold_point)
            if self._mission_last_tick_at is not None:
                dt = min(0.25, max(0.0, now - self._mission_last_tick_at))
                if tracking_error <= self._tracking_error_pause_m:
                    self._mission_elapsed_s = min(self._path_duration_s, self._mission_elapsed_s + dt)
                self._mission_last_tick_at = now
        else:
            self._mission_last_tick_at = None

        raw_progress = min(1.0, self._mission_elapsed_s / self._path_duration_s)
        target, target_yaw, target_index = self._sample_path(
            self._path_points_ned,
            self._path_yaws_ned,
            self._ease_progress(raw_progress),
        )
        dt = 1.0 / self._rate_hz if self._last_target_at is None else max(0.0, now - self._last_target_at)
        if self._last_target_ned is None:
            self._last_target_ned = self._vehicle_position
        target = self._limit_position_step(
            self._last_target_ned, target, dt, self._reference_speed_limit_m_s
        )
        if self._last_target_yaw is not None:
            target_yaw = self._limit_yaw_step(
                self._last_target_yaw, target_yaw, dt, self._reference_yaw_rate_limit_rad_s
            )
        self._last_target_ned = target
        self._last_target_yaw = target_yaw
        self._last_target_at = now
        self._publish_setpoint(timestamp, target, target_yaw)
        self._warmup_ticks += 1
        self._setpoint_count += 1
        if (
            self._request_offboard
            and status_fresh
            and nav_state != offboard_state
            and self._mode_request_attempts < self._max_command_attempts
            and self._warmup_ticks >= 10
            and now - self._last_mode_request_at >= self._command_retry_s
        ):
            # MAV_CMD_DO_SET_MODE: base mode 1, custom mode 6 (Offboard).
            command_type = self._px4_types["VehicleCommand"]
            command = int(getattr(command_type, "VEHICLE_CMD_DO_SET_MODE", 176))
            self._publish_command(timestamp, command, 1.0, 6.0)
            self._mode_requested = True
            self._mode_request_attempts += 1
            self._last_mode_request_at = now
            if self._first_mode_request_at == 0.0:
                self._first_mode_request_at = now
            self.get_logger().info(
                f"sent PX4 Offboard mode request attempt {self._mode_request_attempts}/{self._max_command_attempts}"
            )
        if nav_state == offboard_state:
            self._mode_requested = True
        if (
            self._request_arm
            and status_fresh
            and nav_state == offboard_state
            and self._vehicle_armed is not True
            and self._arm_request_attempts < self._max_command_attempts
            and now - self._first_mode_request_at >= 1.0
            and now - self._last_arm_request_at >= self._command_retry_s
        ):
            command_type = self._px4_types["VehicleCommand"]
            command = int(getattr(command_type, "VEHICLE_CMD_COMPONENT_ARM_DISARM", 400))
            self._publish_command(timestamp, command, 1.0)
            self._arm_requested = True
            self._arm_request_attempts += 1
            self._last_arm_request_at = now
            self.get_logger().info(
                f"sent PX4 arm request attempt {self._arm_request_attempts}/{self._max_command_attempts}"
            )

        last_index = len(self._path_points_ned) - 1
        if (
            self._land_on_path_end
            and not self._land_requested
            and raw_progress >= 1.0
            and self._vehicle_position is not None
            and math.dist(self._vehicle_position, self._path_points_ned[-1]) <= self._path_end_tolerance_m
        ):
            self._land_requested = True
            self._last_land_request_at = 0.0
            self.get_logger().info("final ROS reference point reached; requesting PX4 NAV_LAND")

        if self._land_requested and self._land_request_attempts < self._max_command_attempts:
            if now - self._last_land_request_at >= self._command_retry_s:
                command_type = self._px4_types["VehicleCommand"]
                command = int(getattr(command_type, "VEHICLE_CMD_NAV_LAND", 21))
                self._publish_command(timestamp, command)
                self._land_request_attempts += 1
                self._last_land_request_at = now
                self.get_logger().info(
                    f"sent PX4 NAV_LAND request attempt {self._land_request_attempts}/{self._max_command_attempts}"
                )

        last_index = len(self._path_points_ned) - 1
        target = self._path_points_ned[target_index]
        status = (
            f"OUTPUTTING_SETPOINT count={self._setpoint_count} "
            f"progress={raw_progress:.3f} index={target_index}/{last_index} "
            f"local_ned=({self._vehicle_position[0]:.2f},{self._vehicle_position[1]:.2f},{self._vehicle_position[2]:.2f}) "
            f"target_ned=({target[0]:.2f},{target[1]:.2f},{target[2]:.2f})"
        )
        self._publish_status(status)

    def _tick_super(self, now: float, status_fresh: bool, nav_state: int, offboard_state: int) -> None:
        """Publish the latest SUPER PositionCommand through the PX4 boundary."""
        if self._super_path_revoked:
            self._publish_status("SUPER_PATH_REVOKED_OUTPUT_INHIBITED")
            return
        if not self._super_path_ready or self._super_command_enu is None or self._super_command_yaw_enu is None:
            self._publish_status("WAITING_FOR_SUPER_COMMAND")
            return
        if now - self._super_command_received_at > self._super_command_timeout_s:
            self._publish_status("STALE_SUPER_COMMAND_OUTPUT_INHIBITED")
            return
        if self._vehicle_position is None or now - self._vehicle_received_at > self._timesync_timeout_s:
            self._publish_status("WAITING_FOR_VALID_PX4_POSITION")
            return
        timestamp = self._px4_timestamp()
        if timestamp is None:
            self._publish_status("WAITING_FOR_TIMESYNC")
            return

        self._publish_heartbeat(timestamp)
        in_offboard_flight = self._vehicle_armed is True and nav_state == offboard_state
        if in_offboard_flight and self._mission_started_at is None:
            self._mission_started_at = now
        target = self._enu_to_ned(
            self._super_command_enu, self._origin_ned, self._alignment_yaw
        )
        target_yaw = self._enu_yaw_to_ned(self._super_command_yaw_enu, self._alignment_yaw)
        dt = 1.0 / self._rate_hz if self._last_target_at is None else max(0.0, now - self._last_target_at)
        if self._last_target_ned is None:
            self._last_target_ned = self._vehicle_position
        target = self._limit_position_step(
            self._last_target_ned, target, dt, self._reference_speed_limit_m_s
        )
        if self._last_target_yaw is not None:
            target_yaw = self._limit_yaw_step(
                self._last_target_yaw, target_yaw, dt, self._reference_yaw_rate_limit_rad_s
            )
        self._last_target_ned = target
        self._last_target_yaw = target_yaw
        self._last_target_at = now
        self._publish_setpoint(timestamp, target, target_yaw)
        self._warmup_ticks += 1
        self._setpoint_count += 1

        if (
            self._request_offboard
            and status_fresh
            and nav_state != offboard_state
            and self._mode_request_attempts < self._max_command_attempts
            and self._warmup_ticks >= 10
            and now - self._last_mode_request_at >= self._command_retry_s
        ):
            command_type = self._px4_types["VehicleCommand"]
            command = int(getattr(command_type, "VEHICLE_CMD_DO_SET_MODE", 176))
            self._publish_command(timestamp, command, 1.0, 6.0)
            self._mode_requested = True
            self._mode_request_attempts += 1
            self._last_mode_request_at = now
            if self._first_mode_request_at == 0.0:
                self._first_mode_request_at = now
            self.get_logger().info(
                f"sent PX4 Offboard mode request attempt {self._mode_request_attempts}/{self._max_command_attempts}"
            )
        if nav_state == offboard_state:
            self._mode_requested = True
        if (
            self._request_arm
            and status_fresh
            and nav_state == offboard_state
            and self._vehicle_armed is not True
            and self._arm_request_attempts < self._max_command_attempts
            and now - self._first_mode_request_at >= 1.0
            and now - self._last_arm_request_at >= self._command_retry_s
        ):
            command_type = self._px4_types["VehicleCommand"]
            command = int(getattr(command_type, "VEHICLE_CMD_COMPONENT_ARM_DISARM", 400))
            self._publish_command(timestamp, command, 1.0)
            self._arm_requested = True
            self._arm_request_attempts += 1
            self._last_arm_request_at = now
            self.get_logger().info(
                f"sent PX4 arm request attempt {self._arm_request_attempts}/{self._max_command_attempts}"
            )

        if (
            self._land_on_super_mission_complete
            and self._super_mission_complete
            and not self._land_requested
        ):
            self._land_requested = True
            self._last_land_request_at = 0.0
            self.get_logger().info("SUPER mission complete; requesting PX4 NAV_LAND")
        if self._land_requested and self._land_request_attempts < self._max_command_attempts:
            if now - self._last_land_request_at >= self._command_retry_s:
                command_type = self._px4_types["VehicleCommand"]
                command = int(getattr(command_type, "VEHICLE_CMD_NAV_LAND", 21))
                self._publish_command(timestamp, command)
                self._land_request_attempts += 1
                self._last_land_request_at = now
                self.get_logger().info(
                    f"sent PX4 NAV_LAND request attempt {self._land_request_attempts}/{self._max_command_attempts}"
                )

        status = (
            f"OUTPUTTING_SETPOINT count={self._setpoint_count} source=SUPER "
            f"local_ned=({self._vehicle_position[0]:.2f},{self._vehicle_position[1]:.2f},{self._vehicle_position[2]:.2f}) "
            f"target_ned=({target[0]:.2f},{target[1]:.2f},{target[2]:.2f})"
        )
        self._publish_status(status)

    def _publish_status(self, text: str) -> None:
        now = time.monotonic()
        if (
            text.startswith("OUTPUTTING_SETPOINT")
            and self._last_status.startswith("OUTPUTTING_SETPOINT")
            and now - self._last_status_at < 1.0
        ) or (text == self._last_status and now - self._last_status_at < 1.0):
            return
        self._last_status = text
        self._last_status_at = now
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = Px4OffboardBridge()
    except (ImportError, RuntimeError, ValueError) as exc:
        print(f"px4_offboard_bridge cannot start: {exc}")
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
