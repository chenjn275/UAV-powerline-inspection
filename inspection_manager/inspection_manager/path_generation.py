"""Pure-Python reference-path generators used by the inspection manager.

Keeping the geometry independent of ROS makes it possible to validate the
vertical tower trajectory in a small offline unit test as well as from the
running node.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple


# (x, y, z, yaw), in the configured planning frame.
VerticalTowerSample = Tuple[float, float, float, float]


@dataclass(frozen=True)
class TowerScanPlan:
    center_x: float
    center_y: float
    base_z: float
    top_z: float
    footprint_radius: float
    orbit_radius: float
    z_min: float
    z_max: float
    samples: Tuple[VerticalTowerSample, ...]
    minimum_tower_clearance: float


def plan_observed_tower_scan(
    *,
    center_x: float,
    center_y: float,
    base_z: float,
    top_z: float,
    footprint_radius: float,
    minimum_orbit_radius: float = 3.0,
    surface_standoff: float = 2.0,
    base_clearance: float = 1.5,
    top_clearance: float = 1.0,
    orbit_count: float = 2.0,
    samples_per_orbit: int = 72,
    start_angle: float = 0.0,
) -> TowerScanPlan:
    """Create a rising orbit using measured tower bounds and margins.

    The point-cloud estimator's horizontal spread is treated as a conservative
    tower footprint proxy. The orbit is expanded by a configurable surface
    standoff; this only checks clearance from the observed tower itself and
    does not replace surrounding-obstacle detection.
    """
    numeric = (center_x, center_y, base_z, top_z, footprint_radius,
               minimum_orbit_radius, surface_standoff, base_clearance, top_clearance)
    if not all(math.isfinite(float(value)) for value in numeric):
        raise ValueError("tower scan geometry must be finite")
    if footprint_radius < 0.0 or minimum_orbit_radius <= 0.0 or surface_standoff < 0.0:
        raise ValueError("tower radii and standoff are outside valid bounds")
    if base_clearance < 0.0 or top_clearance < 0.0:
        raise ValueError("vertical clearances must be non-negative")
    z_min = float(base_z) + float(base_clearance)
    z_max = float(top_z) - float(top_clearance)
    if z_max <= z_min:
        raise ValueError("observed tower is too short for requested vertical clearances")
    orbit_radius = max(float(minimum_orbit_radius), float(footprint_radius) + float(surface_standoff))
    samples = tuple(generate_vertical_tower_path(
        center_x=float(center_x), center_y=float(center_y), z_min=z_min, z_max=z_max,
        orbit_radius=orbit_radius, orbit_count=orbit_count,
        samples_per_orbit=samples_per_orbit, start_angle=start_angle,
    ))
    clearance = min(
        math.hypot(sample[0] - center_x, sample[1] - center_y) - footprint_radius
        for sample in samples
    )
    if clearance + 1e-9 < surface_standoff:
        raise ValueError("generated path violates tower surface standoff")
    return TowerScanPlan(
        center_x=float(center_x), center_y=float(center_y), base_z=float(base_z), top_z=float(top_z),
        footprint_radius=float(footprint_radius), orbit_radius=orbit_radius,
        z_min=z_min, z_max=z_max, samples=samples, minimum_tower_clearance=clearance,
    )


def generate_vertical_tower_path(
    center_x: float,
    center_y: float,
    z_min: float,
    z_max: float,
    orbit_radius: float,
    orbit_count: float = 2.0,
    samples_per_orbit: int = 72,
    start_angle: float = 0.0,
) -> List[VerticalTowerSample]:
    """Generate a constant-radius rising helix around a tower.

    The path starts at ``z_min`` and ends at ``z_max`` while completing the
    requested number of counter-clockwise turns.  At least two turns are
    always generated so a default task visibly inspects every side of the
    tower.  The returned samples include both endpoints and carry a yaw that
    is tangent to the orbit, suitable for a vehicle that should look along
    its direction of travel.

    ``ValueError`` is raised for non-finite coordinates, a non-positive radius,
    or an inverted altitude interval.  A requested orbit count below two is
    clamped to two by design; callers cannot accidentally configure a partial
    inspection.
    """

    values = (center_x, center_y, z_min, z_max, orbit_radius, orbit_count)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("vertical tower parameters must be finite")
    if orbit_radius <= 0.0:
        raise ValueError("orbit_radius must be positive")
    if z_max <= z_min:
        raise ValueError("z_max must be greater than z_min")
    if not math.isfinite(float(start_angle)):
        raise ValueError("start_angle must be finite")

    samples_per_orbit = max(8, int(samples_per_orbit))
    turns = max(2.0, float(orbit_count))
    sample_count = int(math.ceil(turns * samples_per_orbit)) + 1

    samples: List[VerticalTowerSample] = []
    for index in range(sample_count):
        progress = index / float(sample_count - 1)
        angle = float(start_angle) + (2.0 * math.pi * turns * progress)
        samples.append(
            (
                float(center_x) + float(orbit_radius) * math.cos(angle),
                float(center_y) + float(orbit_radius) * math.sin(angle),
                float(z_min) + (float(z_max) - float(z_min)) * progress,
                angle + math.pi / 2.0,
            )
        )
    return samples
