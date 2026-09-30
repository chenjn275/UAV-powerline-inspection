"""Regression tests for guarded PX4 SITL arming retries."""

from __future__ import annotations

import importlib.util
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


SCRIPT = Path(__file__).with_name("px4_mavlink_inspection.py")
SPEC = importlib.util.spec_from_file_location("px4_mavlink_inspection", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
CONTROLLER_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CONTROLLER_MODULE)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


class FakeMessage:
    def __init__(self, message_type: str, **fields: object) -> None:
        self.message_type = message_type
        for name, value in fields.items():
            setattr(self, name, value)

    def get_type(self) -> str:
        return self.message_type


class FakeMav:
    def __init__(self, link: "FakeLink") -> None:
        self.link = link
        self.arm_requests = 0
        self.parameter_sets: list[tuple[object, ...]] = []

    def set_position_target_local_ned_send(self, *args: object) -> None:
        pass

    def param_set_send(self, *args: object) -> None:
        self.parameter_sets.append(args)

    def command_long_send(self, *args: object) -> None:
        self.arm_requests += 1
        result = (
            CONTROLLER_MODULE.mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED
            if self.arm_requests == 1
            else CONTROLLER_MODULE.mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        self.link.messages.append(
            FakeMessage(
                "COMMAND_ACK",
                command=CONTROLLER_MODULE.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                result=result,
                result_param2=0,
            )
        )
        if result == CONTROLLER_MODULE.mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.link.messages.append(
                FakeMessage(
                    "HEARTBEAT",
                    base_mode=CONTROLLER_MODULE.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                    custom_mode=6,
                )
            )


class FakeLink:
    def __init__(self) -> None:
        self.target_system = 1
        self.target_component = 1
        self.messages: deque[FakeMessage] = deque()
        self.mav = FakeMav(self)

    def recv_match(self, blocking: bool = False) -> FakeMessage | None:
        return self.messages.popleft() if self.messages else None


class ArmingRetryTests(unittest.TestCase):
    def test_controller_never_enables_arm_without_gps(self) -> None:
        controller = object.__new__(CONTROLLER_MODULE.Px4InspectionController)
        controller.link = FakeLink()
        controller._disallow_arm_without_gps()

        self.assertEqual(controller.link.mav.parameter_sets[0][2], b"COM_ARM_WO_GPS")
        self.assertEqual(controller.link.mav.parameter_sets[0][3], 0.0)
        self.assertEqual(
            controller.link.mav.parameter_sets[0][4],
            CONTROLLER_MODULE.mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )

    def test_temporarily_rejected_arm_is_retried_until_accepted(self) -> None:
        controller = object.__new__(CONTROLLER_MODULE.Px4InspectionController)
        controller.args = SimpleNamespace(rate_hz=20.0)
        controller.link = FakeLink()
        controller.last_position = None
        controller.last_mode = "unknown"
        controller.armed = None
        controller.arm_ack = None
        controller.arm_ack_sequence = 0
        controller.status_text = ""
        controller.preflight_ready = False
        controller.last_log = 0.0
        controller._request_arm()

        clock = FakeClock()
        with (
            patch.object(CONTROLLER_MODULE.time, "monotonic", clock.monotonic),
            patch.object(CONTROLLER_MODULE.time, "sleep", clock.sleep),
        ):
            controller._wait_for_armed((0.0, 0.0, -2.0))

        self.assertEqual(controller.link.mav.arm_requests, 2)
        self.assertTrue(controller.armed)

    def test_permanently_denied_arm_fails_without_retry(self) -> None:
        controller = object.__new__(CONTROLLER_MODULE.Px4InspectionController)
        controller.args = SimpleNamespace(rate_hz=20.0)
        controller.link = FakeLink()
        controller.last_position = None
        controller.last_mode = "unknown"
        controller.armed = None
        controller.arm_ack = None
        controller.arm_ack_sequence = 0
        controller.status_text = "Preflight check failed"
        controller.preflight_ready = True
        controller.last_log = 0.0
        controller._request_arm()
        controller.link.messages.clear()
        controller.link.messages.append(
            FakeMessage(
                "COMMAND_ACK",
                command=CONTROLLER_MODULE.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                result=CONTROLLER_MODULE.mavutil.mavlink.MAV_RESULT_DENIED,
                result_param2=17,
            )
        )

        with self.assertRaisesRegex(RuntimeError, "denied arming.*Preflight check failed"):
            controller._wait_for_armed((0.0, 0.0, -2.0))
        self.assertEqual(controller.link.mav.arm_requests, 1)


if __name__ == "__main__":
    unittest.main()
