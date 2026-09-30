"""Known-line geometry and conservative circular-scan generation.

The implementation intentionally uses only the Python standard library.  It is
the deterministic reference used by later ROS nodes; ROS messages and SUPER
interfaces should wrap these values instead of duplicating the geometry.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


Vec3 = tuple[float, float, float]


def _vec(value: Sequence[float]) -> Vec3:
    if len(value) != 3:
        raise ValueError("a 3-D vector must contain exactly three values")
    return (float(value[0]), float(value[1]), float(value[2]))


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a: Vec3, factor: float) -> Vec3:
    return (a[0] * factor, a[1] * factor, a[2] * factor)


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a: Vec3) -> float:
    return math.sqrt(_dot(a, a))


def _unit(a: Vec3, *, name: str = "vector") -> Vec3:
    length = _norm(a)
    if length <= 1e-12:
        raise ValueError(f"{name} must be non-zero")
    return _scale(a, 1.0 / length)


def _distance(a: Vec3, b: Vec3) -> float:
    return _norm(_sub(a, b))


def _rotate(vector: Vec3, axis: Vec3, angle: float) -> Vec3:
    """Rotate a vector with Rodrigues' formula."""

    axis = _unit(axis, name="rotation axis")
    c = math.cos(angle)
    s = math.sin(angle)
    return _add(
        _add(_scale(vector, c), _scale(_cross(axis, vector), s)),
        _scale(axis, _dot(axis, vector) * (1.0 - c)),
    )


@dataclass(frozen=True)
class LineSample:
    s: float
    position: Vec3
    tangent: Vec3
    normal: Vec3
    binormal: Vec3


class PolylineLineModel:
    """Piecewise-linear centerline with a parallel-transport frame.

    A Frenet frame is undefined on a straight span, so the model transports a
    single initial normal through each tangent change.  The resulting frame is
    stable on straight lines and does not silently jump when curvature tends to
    zero.
    """

    def __init__(self, points: Iterable[Sequence[float]], *, model_version: str = "unversioned"):
        self.points = tuple(_vec(point) for point in points)
        if len(self.points) < 2:
            raise ValueError("a line model needs at least two points")

        self.model_version = str(model_version)
        self._segment_lengths = tuple(
            _distance(self.points[i + 1], self.points[i])
            for i in range(len(self.points) - 1)
        )
        if any(length <= 1e-9 for length in self._segment_lengths):
            raise ValueError("line model contains coincident consecutive points")
        cumulative = [0.0]
        for length in self._segment_lengths:
            cumulative.append(cumulative[-1] + length)
        self._cumulative = tuple(cumulative)
        self._tangents = tuple(
            _unit(_sub(self.points[i + 1], self.points[i]), name="line segment")
            for i in range(len(self.points) - 1)
        )
        self._normals, self._binormals = self._build_parallel_transport_frames()

    @property
    def length(self) -> float:
        return self._cumulative[-1]

    @property
    def segment_count(self) -> int:
        return len(self._segment_lengths)

    def _build_parallel_transport_frames(self) -> tuple[tuple[Vec3, ...], tuple[Vec3, ...]]:
        tangent = self._tangents[0]
        references = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        reference = min(references, key=lambda candidate: abs(_dot(candidate, tangent)))
        normal = _unit(_cross(reference, tangent), name="initial line normal")
        binormal = _unit(_cross(tangent, normal), name="initial line binormal")
        normals = [normal]
        binormals = [binormal]

        for previous, current in zip(self._tangents, self._tangents[1:]):
            axis = _cross(previous, current)
            sin_angle = _norm(axis)
            cos_angle = max(-1.0, min(1.0, _dot(previous, current)))
            if sin_angle > 1e-10:
                normal = _rotate(normal, axis, math.atan2(sin_angle, cos_angle))
            elif cos_angle < 0.0:
                # A 180-degree reversal has no unique minimal rotation.  Keep
                # the old normal and rebuild the binormal from the new tangent.
                normal = _scale(normal, -1.0)
            normal = _unit(_sub(normal, _scale(current, _dot(normal, current))), name="transported normal")
            binormal = _unit(_cross(current, normal), name="transported binormal")
            normals.append(normal)
            binormals.append(binormal)
        return tuple(normals), tuple(binormals)

    def _segment_for_s(self, s: float) -> tuple[int, float]:
        if not math.isfinite(s):
            raise ValueError("arc-length query must be finite")
        clamped = max(0.0, min(self.length, float(s)))
        if clamped >= self.length:
            index = self.segment_count - 1
            return index, 1.0
        for index, (start, end) in enumerate(zip(self._cumulative, self._cumulative[1:])):
            if clamped <= end:
                return index, (clamped - start) / (end - start)
        raise RuntimeError("failed to locate line segment")

    def sample(self, s: float) -> LineSample:
        index, fraction = self._segment_for_s(s)
        start = self.points[index]
        end = self.points[index + 1]
        position = _add(start, _scale(_sub(end, start), fraction))
        return LineSample(
            s=max(0.0, min(self.length, float(s))),
            position=position,
            tangent=self._tangents[index],
            normal=self._normals[index],
            binormal=self._binormals[index],
        )

    def nearest_distance(self, point: Sequence[float]) -> float:
        """Return the distance from a point to the complete centerline."""

        query = _vec(point)
        best = math.inf
        for start, end in zip(self.points, self.points[1:]):
            segment = _sub(end, start)
            denominator = _dot(segment, segment)
            fraction = max(0.0, min(1.0, _dot(_sub(query, start), segment) / denominator))
            closest = _add(start, _scale(segment, fraction))
            best = min(best, _distance(query, closest))
        return best


@dataclass(frozen=True)
class ArcScanSpec:
    start_s: float
    end_s: float
    radius: float
    theta_center: float = 0.0
    theta_amplitude: float = math.radians(10.0)
    period: float = 16.0
    along_line_speed: float = 0.5
    sample_dt: float = 0.05

    def validate(self, line_length: float) -> None:
        if not (0.0 <= self.start_s <= line_length and 0.0 <= self.end_s <= line_length):
            raise ValueError("scan arc-length bounds must lie inside the line model")
        if self.radius <= 0.0:
            raise ValueError("scan radius must be positive")
        if self.period <= 0.0 or self.along_line_speed <= 0.0 or self.sample_dt <= 0.0:
            raise ValueError("period, speed, and sample_dt must be positive")
        if abs(self.theta_amplitude) >= math.pi:
            raise ValueError("scan angle amplitude must be less than 180 degrees")


@dataclass(frozen=True)
class ArcSample:
    time: float
    s: float
    theta: float
    position: Vec3
    line_position: Vec3
    tangent: Vec3
    normal: Vec3
    binormal: Vec3


def generate_arc_scan(line: PolylineLineModel, spec: ArcScanSpec) -> tuple[ArcSample, ...]:
    """Generate a time-parameterized scan with a fixed cross-section radius."""

    spec.validate(line.length)
    distance = abs(spec.end_s - spec.start_s)
    duration = distance / spec.along_line_speed
    count = max(1, int(math.ceil(duration / spec.sample_dt)))
    direction = 1.0 if spec.end_s >= spec.start_s else -1.0
    samples = []
    for index in range(count + 1):
        time = min(duration, index * spec.sample_dt)
        s = spec.start_s + direction * spec.along_line_speed * time
        if index == count:
            s = spec.end_s
        frame = line.sample(s)
        theta = spec.theta_center + spec.theta_amplitude * math.sin(2.0 * math.pi * time / spec.period)
        radial = _add(_scale(frame.normal, math.cos(theta)), _scale(frame.binormal, math.sin(theta)))
        position = _add(frame.position, _scale(radial, spec.radius))
        samples.append(
            ArcSample(
                time=time,
                s=s,
                theta=theta,
                position=position,
                line_position=frame.position,
                tangent=frame.tangent,
                normal=frame.normal,
                binormal=frame.binormal,
            )
        )
    return tuple(samples)


@dataclass(frozen=True)
class SphereObstacle:
    center: Vec3
    radius: float
    name: str = "obstacle"

    def __post_init__(self) -> None:
        if self.radius < 0.0:
            raise ValueError("obstacle radius cannot be negative")

    def clearance(self, point: Sequence[float]) -> float:
        return _distance(_vec(point), self.center) - self.radius


@dataclass(frozen=True)
class ClearanceReport:
    accepted: bool
    min_line_distance: float
    min_obstacle_clearance: float
    first_violation_index: int | None
    violation_reason: str | None


def validate_clearance(
    line: PolylineLineModel,
    trajectory: Iterable[Sequence[float]],
    *,
    line_exclusion_radius: float,
    obstacles: Iterable[SphereObstacle] = (),
    obstacle_margin: float = 0.0,
) -> ClearanceReport:
    """Check every supplied trajectory sample against full line geometry.

    The caller controls the sampling interval.  Production integration must
    choose it from velocity/acceleration bounds or replace this with a
    continuous swept-volume checker before declaring a trajectory safe.
    """

    if line_exclusion_radius < 0.0 or obstacle_margin < 0.0:
        raise ValueError("clearance radii and margins cannot be negative")
    points = tuple(_vec(point) for point in trajectory)
    if not points:
        raise ValueError("trajectory must contain at least one point")
    obstacle_list = tuple(obstacles)
    min_line = math.inf
    min_obstacle = math.inf
    first_violation = None
    reason = None
    for index, point in enumerate(points):
        line_distance = line.nearest_distance(point)
        min_line = min(min_line, line_distance)
        if line_distance < line_exclusion_radius and first_violation is None:
            first_violation = index
            reason = "line_exclusion"
        for obstacle in obstacle_list:
            clearance = obstacle.clearance(point)
            min_obstacle = min(min_obstacle, clearance)
            if clearance < obstacle_margin and first_violation is None:
                first_violation = index
                reason = f"obstacle:{obstacle.name}"
    return ClearanceReport(
        accepted=first_violation is None,
        min_line_distance=min_line,
        min_obstacle_clearance=min_obstacle if obstacle_list else math.inf,
        first_violation_index=first_violation,
        violation_reason=reason,
    )
