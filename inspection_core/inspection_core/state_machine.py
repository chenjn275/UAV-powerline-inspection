"""Small, deterministic inspection task state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .health import HealthState


class InspectionState(str, Enum):
    INITIALIZING = "INITIALIZING"
    WAITING_FOR_OBSERVATION = "WAITING_FOR_OBSERVATION"
    SCANNING = "SCANNING"
    BRAKING = "BRAKING"
    RECONNECTING = "RECONNECTING"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"


@dataclass
class InspectionTask:
    state: InspectionState = InspectionState.INITIALIZING
    coverage_ratio: float = 0.0
    has_safe_candidate: bool = False
    reconnect_available: bool = False

    def initialize(self, *, has_safe_candidate: bool) -> InspectionState:
        self.has_safe_candidate = has_safe_candidate
        self.state = (InspectionState.WAITING_FOR_OBSERVATION if has_safe_candidate
                      else InspectionState.ABORTED)
        return self.state

    def observe(self, *, valid: bool, coverage_ratio: float) -> InspectionState:
        self.coverage_ratio = max(0.0, min(1.0, float(coverage_ratio)))
        if self.state == InspectionState.WAITING_FOR_OBSERVATION and valid:
            self.state = InspectionState.SCANNING
        return self.state

    def update_health(self, health: HealthState) -> InspectionState:
        if health == HealthState.READY:
            return self.state
        if self.state in (InspectionState.COMPLETE, InspectionState.ABORTED):
            return self.state
        self.state = InspectionState.BRAKING
        return self.state

    def reconnect(self, *, available: bool) -> InspectionState:
        self.reconnect_available = available
        if self.state != InspectionState.BRAKING:
            return self.state
        self.state = InspectionState.RECONNECTING if available else InspectionState.ABORTED
        return self.state

    def resume(self, *, valid_observation: bool) -> InspectionState:
        if self.state == InspectionState.RECONNECTING and valid_observation:
            self.state = InspectionState.SCANNING
        return self.state

    def finish(self) -> InspectionState:
        if self.coverage_ratio >= 0.95:
            self.state = InspectionState.COMPLETE
        return self.state
