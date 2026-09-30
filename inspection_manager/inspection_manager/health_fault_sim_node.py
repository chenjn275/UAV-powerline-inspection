#!/usr/bin/env python3
"""Deterministic health/fault injection for the inspection simulation.

The node emits the same small text contract accepted by
``inspection_manager_node`` and cycles through healthy operation, a stale
point-cloud fault, a link fault, and recovery.  It is intentionally separate
from the sensor simulator so each fault can be reproduced or replaced by a
real supervisor without changing the planner.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from inspection_core.health import HealthState


class HealthFaultSim(Node):
    """Publish a configurable repeating health/fault sequence."""

    def __init__(self) -> None:
        super().__init__("health_fault_sim")
        self.declare_parameter(
            "sequence",
            "READY:5,STALE_POINTCLOUD:3,READY:5,LINK_LOST:3,READY:5",
        )
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("topic", "/inspection/health")
        self._sequence = self._parse_sequence(str(self.get_parameter("sequence").value))
        self._period = max(0.01, 1.0 / float(self.get_parameter("rate").value))
        self._started = time.monotonic()
        self._pub = self.create_publisher(
            String, str(self.get_parameter("topic").value), 10
        )
        self.create_timer(self._period, self._publish)

    @staticmethod
    def _parse_sequence(value: str) -> tuple[tuple[HealthState, float], ...]:
        parsed = []
        for item in value.split(","):
            state_text, separator, duration_text = item.strip().partition(":")
            if not separator:
                continue
            try:
                state = HealthState(state_text.strip().upper())
                duration = float(duration_text)
            except (TypeError, ValueError):
                continue
            if duration > 0.0:
                parsed.append((state, duration))
        if not parsed:
            parsed.append((HealthState.READY, 1.0))
        return tuple(parsed)

    def _current_state(self) -> HealthState:
        elapsed = (time.monotonic() - self._started) % sum(duration for _, duration in self._sequence)
        for state, duration in self._sequence:
            if elapsed < duration:
                return state
            elapsed -= duration
        return self._sequence[-1][0]

    @staticmethod
    def _action(state: HealthState) -> str:
        return {
            HealthState.READY: "ALLOW_CURRENT_TRAJECTORY",
            HealthState.STALE_POINTCLOUD: "PAUSE_AND_REPLAN",
            HealthState.STALE_TRAJECTORY: "BRAKE_OR_VERIFIED_BACKUP",
            HealthState.STALE_MODEL: "PAUSE_AND_REPLAN",
            HealthState.LINK_LOST: "HOLD_ONLY_IF_INDEPENDENT_ESTIMATE_VALID",
            HealthState.MULTIPLE_FAULTS: "BRAKE_AND_REQUIRE_SUPERVISOR_DECISION",
        }[state]

    def _publish(self) -> None:
        state = self._current_state()
        msg = String()
        msg.data = f"HEALTH state={state.value} action={self._action(state)}"
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HealthFaultSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
