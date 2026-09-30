#!/usr/bin/env python3
"""Drive the PX4 Classic inspection model through a simulated tower scan.

This is a simulation-only MAVLink adapter for hosts where the PX4-matched
``px4_msgs``/Micro XRCE-DDS chain is not installed yet.  It sends the same
vertical rising-helix reference used by ``inspection_manager`` as
``SET_POSITION_TARGET_LOCAL_NED`` messages to PX4 SITL.  The ROS Offboard
bridge remains the production integration boundary and stays disabled by
default.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from typing import Optional

try:
    from pymavlink import mavutil
except ImportError as exc:  # pragma: no cover - exercised on an unprepared host
    raise SystemExit(
        "pymavlink is required for this SITL-only adapter; install it in the "
        "active Python environment before running the PX4 inspection demo"
    ) from exc


# Ignore velocity, acceleration, yaw, and yaw rate while commanding position.
# Do not set FORCE_SET (bit 9): this message does not command acceleration.
POSITION_ONLY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)
STOP_REQUESTED = False


def _stop(_signum: int, _frame: object) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--connection",
        default="udpin:0.0.0.0:14540",
        help="pymavlink connection string (default: %(default)s)",
    )
    parser.add_argument("--center-north", type=float, default=0.0)
    parser.add_argument("--center-east", type=float, default=0.0)
    parser.add_argument("--radius", type=float, default=4.0)
    parser.add_argument("--z-min", type=float, default=1.5)
    parser.add_argument("--z-max", type=float, default=9.5)
    parser.add_argument("--orbits", type=float, default=2.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument(
        "--no-arm",
        action="store_true",
        help="stream references without sending the arm command",
    )
    parser.add_argument(
        "--no-land",
        action="store_true",
        help="leave landing to the operator when the reference ends",
    )
    return parser


def _validate(args: argparse.Namespace) -> None:
    values = (
        args.center_north,
        args.center_east,
        args.radius,
        args.z_min,
        args.z_max,
        args.orbits,
        args.rate_hz,
        args.duration,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("all trajectory parameters must be finite")
    if args.radius <= 0.0:
        raise ValueError("--radius must be positive")
    if args.z_max <= args.z_min:
        raise ValueError("--z-max must be greater than --z-min")
    if args.orbits < 2.0:
        raise ValueError("--orbits must be at least 2 for a complete inspection")
    if args.rate_hz < 2.0:
        raise ValueError("--rate-hz must be at least 2 Hz for PX4 Offboard")
    if args.duration <= 0.0:
        raise ValueError("--duration must be positive")
    if args.no_arm and not args.no_land:
        raise ValueError("--no-arm requires --no-land so no flight mode is requested")


class Px4InspectionController:
    def __init__(self, connection: str, args: argparse.Namespace) -> None:
        self.args = args
        self.link = mavutil.mavlink_connection(connection, autoreconnect=True)
        self.last_position: Optional[tuple[float, float, float]] = None
        self.last_mode = "unknown"
        self.armed: Optional[bool] = None
        self.arm_ack: Optional[tuple[int, int]] = None
        self.arm_ack_sequence = 0
        self.status_text = ""
        self.preflight_ready = False
        self.last_log = 0.0

    def wait_for_heartbeat(self) -> None:
        print("waiting for PX4 heartbeat", flush=True)
        heartbeat = self.link.wait_heartbeat(timeout=120.0)
        if heartbeat is None:
            raise RuntimeError("PX4 heartbeat timeout after 120 seconds; is SITL running on UDP 14540?")
        print(
            f"PX4 connected system={self.link.target_system} "
            f"component={self.link.target_component}",
            flush=True,
        )

    def _send_position(self, north: float, east: float, down: float) -> None:
        self.link.mav.set_position_target_local_ned_send(
            0,
            self.link.target_system,
            self.link.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            POSITION_ONLY_MASK,
            float(north),
            float(east),
            float(down),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def _read_feedback(self) -> None:
        while True:
            message = self.link.recv_match(blocking=False)
            if message is None:
                return
            message_type = message.get_type()
            if message_type == "LOCAL_POSITION_NED":
                self.last_position = (
                    float(message.x),
                    float(message.y),
                    float(message.z),
                )
            elif message_type == "HEARTBEAT":
                self.last_mode = str(getattr(message, "custom_mode", "unknown"))
                armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                self.armed = bool(int(message.base_mode) & armed_flag)
            elif message_type == "COMMAND_ACK":
                if int(getattr(message, "command", -1)) == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    self.arm_ack = (
                        int(message.result),
                        int(getattr(message, "result_param2", 0)),
                    )
                    self.arm_ack_sequence += 1
            elif message_type == "STATUSTEXT":
                status = getattr(message, "text", "")
                if isinstance(status, bytes):
                    status = status.decode("utf-8", errors="replace")
                self.status_text = str(status).rstrip("\x00")
                if "Ready for takeoff" in self.status_text:
                    self.preflight_ready = True
                severity = int(getattr(message, "severity", 255))
                if self.status_text and severity <= mavutil.mavlink.MAV_SEVERITY_INFO:
                    print(f"PX4: {self.status_text}", flush=True)

    def _log_progress(self, progress: float, target: tuple[float, float, float]) -> None:
        now = time.monotonic()
        if now - self.last_log < 1.0:
            return
        self.last_log = now
        current = "unavailable" if self.last_position is None else "%.2f,%.2f,%.2f" % self.last_position
        print(
            "inspection progress=%.0f%% target_ned=(%.2f,%.2f,%.2f) "
            "local_ned=(%s)" % (progress * 100.0, *target, current),
            flush=True,
        )

    def _disallow_arm_without_gps(self) -> None:
        # An earlier version sent the integer parameter as float 1.0, which
        # MAVLink interprets as the int32 bit pattern 1065353216.  Explicitly
        # restore the conservative default and never relax PX4's GPS arming
        # check from this simulation controller.
        self.link.mav.param_set_send(
            self.link.target_system,
            self.link.target_component,
            b"COM_ARM_WO_GPS",
            0.0,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )

    def _request_offboard_and_arm(self) -> None:
        self.link.set_mode("OFFBOARD")
        if self.args.no_arm:
            return
        self._request_arm()

    def _wait_for_preflight_ready(self, timeout: float = 60.0) -> None:
        """Let PX4 initialize its EKF before requesting Offboard mode."""
        deadline = time.monotonic() + timeout
        next_log = 0.0
        while time.monotonic() < deadline and not STOP_REQUESTED:
            self._read_feedback()
            if self.preflight_ready:
                print("PX4 preflight checks passed; preparing Offboard", flush=True)
                return
            now = time.monotonic()
            if now >= next_log:
                print("waiting for PX4 EKF and preflight checks", flush=True)
                next_log = now + 2.0
            time.sleep(0.1)
        detail = f"; last PX4 status: {self.status_text}" if self.status_text else ""
        print(
            "PX4 preflight status was not observed; continuing with setpoint "
            "stream and bounded arm retries" + detail,
            flush=True,
        )

    def _request_arm(self) -> None:
        self.arm_ack = None
        self.link.mav.command_long_send(
            self.link.target_system,
            self.link.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def _land(self) -> None:
        if self.args.no_land or self.args.no_arm:
            return
        self.link.mav.command_long_send(
            self.link.target_system,
            self.link.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def _wait_for_armed(self, target: tuple[float, float, float]) -> None:
        # During SITL startup, valid EKF and battery data can arrive shortly
        # after the Offboard setpoint stream starts.  PX4 reports this as a
        # temporary COMMAND_ACK rejection; keep streaming and retry only
        # within a bounded window instead of treating that race as success or
        # bypassing the arming checks.
        timeout = 60.0
        deadline = time.monotonic() + timeout
        period = 1.0 / self.args.rate_hz
        next_arm_request = time.monotonic() + 1.0
        handled_ack_sequence = self.arm_ack_sequence
        while time.monotonic() < deadline and not STOP_REQUESTED:
            self._send_position(*target)
            self._read_feedback()
            if self.armed:
                print("PX4 armed; starting vertical tower scan", flush=True)
                return
            if self.arm_ack_sequence != handled_ack_sequence:
                handled_ack_sequence = self.arm_ack_sequence
                if self.arm_ack is not None:
                    result, reason = self.arm_ack
                    if result == mavutil.mavlink.MAV_RESULT_DENIED:
                        detail = f"; PX4: {self.status_text}" if self.status_text else ""
                        raise RuntimeError(
                            f"PX4 denied arming (reason {reason}){detail}"
                        )
                    if result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                        detail = f"; PX4: {self.status_text}" if self.status_text else ""
                        print(
                            "PX4 is not ready to arm yet; waiting for preflight checks"
                            f"{detail}",
                            flush=True,
                        )
                        next_arm_request = min(next_arm_request, time.monotonic() + 0.5)
                    elif result not in (
                        mavutil.mavlink.MAV_RESULT_ACCEPTED,
                        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                    ):
                        detail = f"; PX4: {self.status_text}" if self.status_text else ""
                        raise RuntimeError(
                            f"PX4 arming command failed (result {result}, reason {reason}){detail}"
                        )
            now = time.monotonic()
            if now >= next_arm_request:
                self._request_arm()
                next_arm_request = now + 1.0
            time.sleep(period)
        detail = f"; last PX4 status: {self.status_text}" if self.status_text else ""
        raise RuntimeError(
            f"PX4 did not arm within {timeout:.0f} seconds; inspection aborted{detail}"
        )

    def _wait_for_disarm(self, timeout: float = 45.0) -> None:
        deadline = time.monotonic() + timeout
        next_log = 0.0
        while time.monotonic() < deadline:
            self._read_feedback()
            if self.armed is False:
                print("PX4 disarmed after landing", flush=True)
                return
            now = time.monotonic()
            if now >= next_log:
                print("waiting for PX4 landing/disarm", flush=True)
                next_log = now + 2.0
            time.sleep(0.1)
        raise RuntimeError(f"PX4 did not disarm within {timeout:.0f} seconds after LAND")

    def run(self) -> None:
        self.wait_for_heartbeat()
        self._disallow_arm_without_gps()
        # Do not enter Offboard while PX4 is still aligning its EKF.  Starting
        # Offboard references during this transient has repeatedly left this
        # SITL instance without a valid local position estimate.
        self._wait_for_preflight_ready()

        # PX4 needs a valid setpoint stream before it accepts OFFBOARD.
        warmup_target = (self.args.center_north, self.args.center_east, -2.0)
        warmup_period = 1.0 / self.args.rate_hz
        for _ in range(max(20, int(self.args.rate_hz * 2.0))):
            self._send_position(*warmup_target)
            time.sleep(warmup_period)
        self._request_offboard_and_arm()
        if not self.args.no_arm:
            self._wait_for_armed(warmup_target)

        start = time.monotonic()
        while not STOP_REQUESTED:
            elapsed = time.monotonic() - start
            progress = min(1.0, elapsed / self.args.duration)
            angle = 2.0 * math.pi * self.args.orbits * progress
            target = (
                self.args.center_north + self.args.radius * math.cos(angle),
                self.args.center_east + self.args.radius * math.sin(angle),
                -(self.args.z_min + (self.args.z_max - self.args.z_min) * progress),
            )
            self._send_position(*target)
            self._read_feedback()
            self._log_progress(progress, target)
            if progress >= 1.0:
                break
            time.sleep(warmup_period)

        self._land()
        if not self.args.no_land and not self.args.no_arm:
            # Once PX4 accepts NAV_LAND, stop the Offboard stream so it cannot
            # compete with the vehicle's landing controller.
            self._wait_for_disarm()
        print("inspection reference complete", flush=True)


def main() -> int:
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    args = _parser().parse_args()
    try:
        _validate(args)
        Px4InspectionController(args.connection, args).run()
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"px4_mavlink_inspection: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
