"""Explicit local-world coordinate alignment primitives."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


Vec3 = tuple[float, float, float]


def _vec(value: Sequence[float]) -> Vec3:
    if len(value) != 3:
        raise ValueError("expected a 3-D vector")
    return (float(value[0]), float(value[1]), float(value[2]))


@dataclass(frozen=True)
class LocalAlignment:
    """Map a continuous local ENU estimate into a PX4 local NED frame.

    ``origin_ned`` is the PX4 position corresponding to the local estimator's
    origin. ``yaw_offset`` is measured counter-clockwise in the horizontal
    plane from local ENU to PX4 NED after axis conversion. It must be measured
    in a dedicated alignment test; it is not inferred from frame names.
    """

    origin_ned: Vec3 = (0.0, 0.0, 0.0)
    yaw_offset: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "origin_ned", _vec(self.origin_ned))
        if not math.isfinite(self.yaw_offset):
            raise ValueError("yaw_offset must be finite")

    def position_enu_to_ned(self, position_enu: Sequence[float]) -> Vec3:
        east, north, up = _vec(position_enu)
        c = math.cos(self.yaw_offset)
        s = math.sin(self.yaw_offset)
        # ENU -> NED: rotate horizontal axes, then invert vertical axis.
        north_rot = c * north - s * east
        east_rot = s * north + c * east
        return (
            self.origin_ned[0] + north_rot,
            self.origin_ned[1] + east_rot,
            self.origin_ned[2] - up,
        )

    def vector_enu_to_ned(self, vector_enu: Sequence[float]) -> Vec3:
        east, north, up = _vec(vector_enu)
        c = math.cos(self.yaw_offset)
        s = math.sin(self.yaw_offset)
        return (c * north - s * east, s * north + c * east, -up)

    def validate_round_trip(self, position_enu: Sequence[float], expected_ned: Sequence[float], tolerance: float = 1e-6) -> bool:
        actual = self.position_enu_to_ned(position_enu)
        expected = _vec(expected_ned)
        return math.dist(actual, expected) <= tolerance
