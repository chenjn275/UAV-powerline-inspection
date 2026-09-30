"""Explicit freshness and link-health decisions for the inspection state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class HealthState(str, Enum):
    READY = "READY"
    STALE_POINTCLOUD = "STALE_POINTCLOUD"
    STALE_TRAJECTORY = "STALE_TRAJECTORY"
    STALE_MODEL = "STALE_MODEL"
    LINK_LOST = "LINK_LOST"
    MULTIPLE_FAULTS = "MULTIPLE_FAULTS"


@dataclass(frozen=True)
class HealthSnapshot:
    now: float
    pointcloud_stamp: float | None
    trajectory_stamp: float | None
    model_stamp: float | None
    link_last_seen: float | None
    pointcloud_timeout: float
    trajectory_timeout: float
    model_timeout: float
    link_timeout: float

    def _stale(self, stamp: float | None, timeout: float) -> bool:
        return stamp is None or self.now - stamp > timeout or self.now < stamp

    @property
    def state(self) -> HealthState:
        faults = []
        if self._stale(self.pointcloud_stamp, self.pointcloud_timeout):
            faults.append(HealthState.STALE_POINTCLOUD)
        if self._stale(self.trajectory_stamp, self.trajectory_timeout):
            faults.append(HealthState.STALE_TRAJECTORY)
        if self._stale(self.model_stamp, self.model_timeout):
            faults.append(HealthState.STALE_MODEL)
        if self._stale(self.link_last_seen, self.link_timeout):
            faults.append(HealthState.LINK_LOST)
        if not faults:
            return HealthState.READY
        if len(faults) > 1:
            return HealthState.MULTIPLE_FAULTS
        return faults[0]

    @property
    def control_action(self) -> str:
        """Return a conservative action label for the supervisor.

        This does not command a vehicle.  The actual action must be selected
        only after checking PX4's independent estimator and a verified backup
        trajectory.
        """
        if self.state == HealthState.READY:
            return "ALLOW_CURRENT_TRAJECTORY"
        if self.state == HealthState.STALE_TRAJECTORY:
            return "BRAKE_OR_VERIFIED_BACKUP"
        if self.state == HealthState.LINK_LOST:
            return "HOLD_ONLY_IF_INDEPENDENT_ESTIMATE_VALID"
        if self.state in (HealthState.STALE_POINTCLOUD, HealthState.STALE_MODEL):
            return "PAUSE_AND_REPLAN"
        return "BRAKE_AND_REQUIRE_SUPERVISOR_DECISION"
