#!/usr/bin/env python3
"""Guarded inspection-path adapter for PX4 Offboard.

The node is deliberately disabled by default.  When enabled, it accepts a
ROS ENU ``nav_msgs/Path``, converts it to PX4 local NED, and publishes the
PX4 heartbeat/setpoint messages only while all of these gates are true:

* a fresh path with the configured frame id is available;
* fresh PX4 local position and TimesyncStatus messages are available; and
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
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


class Px4OffboardBridge(Node):
    def __init__(self) -> None:
        super().__init__("px4_offboard_bridge")
        self.declare_parameter("path_topic", "/inspection/reference_path")
        self.declare_parameter("vehicle_position_topic", "/fmu/out/vehicle_local_position")
        self.declare_parameter("timesync_topic", "/fmu/out/timesync_status")
        self.declare_parameter("status_topic", "/px4_offboard_bridge/status")
        self.declare_parameter("expected_frame", "map")
        self.declare_parameter("fmu_prefix", "/fmu")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("path_timeout_s", 0.5)
        self.declare_parameter("timesync_timeout_s", 1.5)
        self.declare_parameter("enable_setpoints", False)
        self.declare_parameter("request_offboard", False)
        self.declare_parameter("request_arm", False)
        self.declare_parameter("land_on_path_end", False)
        self.declare_parameter("path_end_tolerance_m", 0.75)
        self.declare_parameter("command_retry_s", 2.0)
        self.declare_parameter("max_command_attempts", 5)
        self.declare_parameter("lookahead_points", 1)
        self.declare_parameter("alignment_yaw_rad", 0.0)
        self.declare_parameter("origin_ned", [0.0, 0.0, 0.0])

        self._path_topic = str(self.get_parameter("path_topic").value)
        self._position_topic = str(self.get_parameter("vehicle_position_topic").value)
        self._timesync_topic = str(self.get_parameter("timesync_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._expected_frame = str(self.get_parameter("expected_frame").value)
        self._fmu_prefix = str(self.get_parameter("fmu_prefix").value).rstrip("/") or "/fmu"
        self._rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self._path_timeout_s = max(0.05, float(self.get_parameter("path_timeout_s").value))
        self._timesync_timeout_s = max(0.1, float(self.get_parameter("timesync_timeout_s").value))
        self._enable_setpoints = bool(self.get_parameter("enable_setpoints").value)
        self._request_offboard = bool(self.get_parameter("request_offboard").value)
        self._request_arm = bool(self.get_parameter("request_arm").value)
        self._land_on_path_end = bool(self.get_parameter("land_on_path_end").value)
        self._path_end_tolerance_m = max(0.1, float(self.get_parameter("path_end_tolerance_m").value))
        self._command_retry_s = max(0.25, float(self.get_parameter("command_retry_s").value))
        self._max_command_attempts = max(1, int(self.get_parameter("max_command_attempts").value))
        self._lookahead_points = max(0, int(self.get_parameter("lookahead_points").value))
        self._alignment_yaw = float(self.get_parameter("alignment_yaw_rad").value)
        origin = tuple(float(value) for value in self.get_parameter("origin_ned").value)
        if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
            raise ValueError("origin_ned must contain three finite values")
        if not math.isfinite(self._alignment_yaw):
            raise ValueError("alignment_yaw_rad must be finite")
        self._origin_ned = origin

        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._path_points_ned: Optional[list[tuple[float, float, float]]] = None
        self._path_received_at = 0.0
        self._last_rejected_frame = ""
        self._vehicle_position: Optional[tuple[float, float, float]] = None
        self._vehicle_received_at = 0.0
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
        self._setpoint_count = 0
        self._last_status = ""
        self._last_status_at = 0.0

        self.create_subscription(Path, self._path_topic, self._on_path, 10)
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
        }

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

    def _on_path(self, msg: Path) -> None:
        frame = msg.header.frame_id.strip()
        if frame != self._expected_frame:
            if frame != self._last_rejected_frame:
                self.get_logger().error(
                    f"rejecting path frame '{frame or '<empty>'}'; expected '{self._expected_frame}'"
                )
                self._last_rejected_frame = frame
            self._path_points_ned = None
            self._publish_status("REJECTED_PATH_FRAME")
            return
        points: list[tuple[float, float, float]] = []
        for pose_stamped in msg.poses:
            position = pose_stamped.pose.position
            points.append(
                self._enu_to_ned(
                    (float(position.x), float(position.y), float(position.z)),
                    self._origin_ned,
                    self._alignment_yaw,
                )
            )
        if not points:
            self._path_points_ned = None
            self._publish_status("REJECTED_EMPTY_PATH")
            return
        first_valid_path = self._path_points_ned is None
        self._path_points_ned = points
        self._path_received_at = time.monotonic()
        self._last_rejected_frame = ""
        # The manager republishes the same fresh path at 5 Hz. Resetting the
        # warmup on each copy made the 10-tick Offboard gate unreachable.
        if first_valid_path:
            self._warmup_ticks = 0
        self._publish_status(f"PATH_READY points={len(points)}")

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

    def _nearest_path_index(self) -> int:
        assert self._path_points_ned is not None
        if self._vehicle_position is None:
            return 0
        px, py, pz = self._vehicle_position
        return min(
            range(len(self._path_points_ned)),
            key=lambda index: math.dist(self._path_points_ned[index], (px, py, pz)),
        )

    def _publish_heartbeat(self, timestamp: int) -> None:
        assert self._px4_types is not None
        msg = self._px4_types["OffboardControlMode"]()
        msg.timestamp = timestamp
        for field in ("position", "velocity", "acceleration", "attitude", "body_rate", "thrust_and_torque", "direct_actuator"):
            if hasattr(msg, field):
                setattr(msg, field, field == "position")
        self._px4_publishers["offboard"].publish(msg)

    def _publish_setpoint(self, timestamp: int, point: tuple[float, float, float]) -> None:
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
            msg.yaw = nan
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
        if self._path_points_ned is None:
            self._publish_status("WAITING_FOR_PATH")
            return
        if time.monotonic() - self._path_received_at > self._path_timeout_s:
            self._publish_status("STALE_PATH_OUTPUT_INHIBITED")
            return
        if self._vehicle_position is None or time.monotonic() - self._vehicle_received_at > self._timesync_timeout_s:
            self._publish_status("WAITING_FOR_VALID_PX4_POSITION")
            return
        timestamp = self._px4_timestamp()
        if timestamp is None:
            self._publish_status("WAITING_FOR_TIMESYNC")
            return
        self._publish_heartbeat(timestamp)
        if self._land_requested:
            target_index = len(self._path_points_ned) - 1
        else:
            nearest = self._nearest_path_index()
            target_index = min(len(self._path_points_ned) - 1, nearest + self._lookahead_points)
        self._publish_setpoint(timestamp, self._path_points_ned[target_index])
        self._warmup_ticks += 1
        self._setpoint_count += 1
        now = time.monotonic()
        if (
            self._request_offboard
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
        if (
            self._request_arm
            and self._mode_requested
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
            and target_index == last_index
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
            f"index={target_index}/{last_index} "
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
