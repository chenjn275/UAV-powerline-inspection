#!/usr/bin/env python3
"""Publish a deterministic inspection candidate and reference path.

This node deliberately stops at candidate selection.  It does not publish
flight-control setpoints; the future SUPER adapter remains the sole planning
and control boundary described by the task book.
"""

import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from std_msgs.msg import String
from inspection_core.health import HealthState

from inspection_core import (
    PolylineLineModel,
    SphereObstacle,
    evaluate_candidates,
    select_best_candidate,
    InspectionTask,
    InspectionState,
)

try:
    # Works when installed as a ROS package.
    from inspection_manager.path_generation import plan_observed_tower_scan
except ImportError:  # pragma: no cover - direct source-file execution
    from path_generation import plan_observed_tower_scan


class InspectionManager(Node):
    def __init__(self) -> None:
        super().__init__("inspection_manager")
        self.declare_parameter("frame_id", "map")
        # The project task is a vertical high-voltage-tower inspection.  Keep
        # the line mode available as an explicit parameter for regression
        # scenarios, while making the node's direct invocation agree with the
        # default launch file and configuration.
        self.declare_parameter("scan_mode", "vertical_tower")
        self.declare_parameter("start_s", 0.0)
        self.declare_parameter("end_s", 50.0)
        self.declare_parameter("radius_candidates", [4.0, 5.0, 6.0])
        self.declare_parameter("line_exclusion_radius", 2.0)
        self.declare_parameter("sample_dt", 0.1)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("require_observation", True)
        self.declare_parameter("tower_center_x", 0.0)
        self.declare_parameter("tower_center_y", 0.0)
        self.declare_parameter("z_min", 2.0)
        self.declare_parameter("z_max", 9.5)
        self.declare_parameter("orbit_radius", 4.0)
        self.declare_parameter("orbit_count", 2.0)
        self.declare_parameter("orbit_samples_per_turn", 72)
        self.declare_parameter("orbit_start_angle", 0.0)
        self.declare_parameter("tower_surface_standoff", 2.0)
        self.declare_parameter("minimum_orbit_radius", 3.0)
        self.declare_parameter("tower_base_clearance", 1.5)
        self.declare_parameter("tower_top_clearance", 1.0)
        self.declare_parameter("tower_observation_timeout", 2.0)
        self.declare_parameter("obstacle_topic", "/inspection/obstacles/points")
        self.declare_parameter("obstacle_timeout", 0.5)
        self.declare_parameter("obstacle_margin", 1.0)
        self.declare_parameter("obstacle_max_step", 0.25)
        self.declare_parameter("replan_radii", [3.0, 4.0, 5.0, 6.0])
        self.declare_parameter("use_observation_center", True)
        # Health is an optional simulation/vehicle supervisor input.  The
        # planner remains backwards compatible with launches that do not
        # provide it, while the integrated demo enables the gate below.
        self.declare_parameter("health_topic", "/inspection/health")
        self.declare_parameter("require_health", False)
        self.declare_parameter("health_timeout", 0.75)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._scan_mode = str(self.get_parameter("scan_mode").value).strip().lower()
        self._line = PolylineLineModel(
            [(0.0, 0.0, 10.0), (50.0, 0.0, 10.0)],
            model_version="straight-single-span-v1",
        )
        self._task = InspectionTask()
        self._require_observation = bool(self.get_parameter("require_observation").value)
        self._observation_received = False
        self._observation_pose = None
        self._tower_center_msg = None
        self._tower_extent_msg = None
        self._tower_observation = None
        self._tower_observation_received_at = None
        self._quality_coverage_ratio = 0.0
        self._quality_coverage_valid = False
        self._quality_coverage_received = False
        # A health fault invalidates the last camera-quality sample.  The
        # planner must receive a new valid sample after reconnection before it
        # is allowed to publish the old reference trajectory again.
        self._awaiting_fresh_quality = False
        self._obstacles = ()
        self._obstacle_received_at = 0.0
        # ``False`` means that no valid obstacle sensor frame has arrived yet.
        # Once a frame has been accepted, a stale stream is unsafe: retaining
        # an old obstacle cloud or silently treating it as clear could send a
        # vehicle through a newly appeared obstacle.  An absent sensor stream
        # before the first frame remains a configurable deployment concern and
        # preserves the deterministic offline launch behaviour.
        self._obstacle_seen = False
        self._last_plan_radius = None
        self._health_state = HealthState.READY
        self._health_received_at = None
        self._status_pub = self.create_publisher(String, "inspection/status", 10)
        self._coverage_pub = self.create_publisher(String, "inspection/coverage", 10)
        self._path_pub = self.create_publisher(PathMsg, "inspection/reference_path", 10)
        self.create_subscription(PoseArray, "inspection/perception/targets", self._on_targets, 10)
        self.create_subscription(PoseStamped, "/inspection/tower/center", self._on_tower_center, 10)
        self.create_subscription(Vector3Stamped, "/inspection/tower/extent", self._on_tower_extent, 10)
        self.create_subscription(String, "/inspection/tower/status", self._on_tower_status, 10)
        self.create_subscription(PointCloud2, str(self.get_parameter("obstacle_topic").value), self._on_obstacles, 10)
        self.create_subscription(String, "/inspection/coverage_quality", self._on_quality_coverage, 10)
        self.create_subscription(
            String,
            str(self.get_parameter("health_topic").value),
            self._on_health,
            10,
        )
        requested_rate = float(self.get_parameter("publish_rate").value)
        # A path bridge treats a stale path as unsafe.  Keep tower paths fresh
        # even when an old line-scan config still contains publish_rate=1.
        minimum_rate = 5.0 if self._scan_mode == "vertical_tower" else 0.1
        rate = max(minimum_rate, requested_rate)
        self.create_timer(1.0 / rate, self._publish_candidate)
        self._publish_candidate()

    def _on_targets(self, msg: PoseArray) -> None:
        if msg.header.frame_id != self._frame_id or not msg.poses:
            return
        pose = msg.poses[0].position
        if not all(math.isfinite(float(v)) for v in (pose.x, pose.y, pose.z)):
            return
        self._observation_received = True
        self._observation_pose = (float(pose.x), float(pose.y), float(pose.z))

    @staticmethod
    def _stamp_key(header) -> tuple[int, int, str]:
        return (int(header.stamp.sec), int(header.stamp.nanosec), str(header.frame_id))

    def _on_tower_center(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != self._frame_id:
            return
        position = msg.pose.position
        if not all(math.isfinite(float(v)) for v in (position.x, position.y, position.z)):
            return
        self._tower_center_msg = msg
        self._update_tower_observation()

    def _on_tower_extent(self, msg: Vector3Stamped) -> None:
        if msg.header.frame_id != self._frame_id:
            return
        extent = msg.vector
        if not all(math.isfinite(float(v)) for v in (extent.x, extent.y, extent.z)):
            return
        if extent.x <= 0.0 or extent.y < 0.0:
            return
        self._tower_extent_msg = msg
        self._update_tower_observation()

    def _update_tower_observation(self) -> None:
        center = self._tower_center_msg
        extent = self._tower_extent_msg
        if center is None or extent is None or self._stamp_key(center.header) != self._stamp_key(extent.header):
            return
        height = float(extent.vector.x)
        midpoint = float(center.pose.position.z)
        self._tower_observation = {
            "center_x": float(center.pose.position.x),
            "center_y": float(center.pose.position.y),
            "base_z": midpoint - height / 2.0,
            "top_z": midpoint + height / 2.0,
            # The current estimator publishes robust horizontal point spread
            # here. Treat it as a conservative footprint proxy for standoff.
            "footprint_radius": float(extent.vector.y),
        }
        self._observation_received = True
        self._observation_pose = (
            self._tower_observation["center_x"],
            self._tower_observation["center_y"],
            float(center.pose.position.z),
        )
        self._tower_observation_received_at = time.monotonic()

    def _on_tower_status(self, msg: String) -> None:
        if msg.data != "VALID":
            self._tower_observation = None
            self._tower_center_msg = None
            self._tower_extent_msg = None
            self._observation_received = False
            self._tower_observation_received_at = None

    def _on_obstacles(self, msg: PointCloud2) -> None:
        # Planning is performed in one fixed frame.  A cloud in another frame
        # is not safe to use, and must not refresh the watchdog timestamp.
        if str(getattr(getattr(msg, "header", None), "frame_id", "")) != self._frame_id:
            return
        names = {str(field.name) for field in getattr(msg, "fields", ())}
        if not {"x", "y", "z"}.issubset(names):
            return
        try:
            rows = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        except (TypeError, ValueError, RuntimeError):
            return

        # sensor_msgs_py changed between ROS 2 releases: Humble commonly
        # returns a structured NumPy array, while older/newer releases and
        # mocks return a generator of tuples/mappings.  Avoid assuming
        # ``reshape`` or named indexing and accept both forms explicitly.
        if hasattr(rows, "reshape"):
            try:
                rows = rows.reshape(-1)
            except (TypeError, ValueError):
                pass
        points = []
        for row in rows:
            try:
                if hasattr(row, "dtype") and getattr(row.dtype, "names", None):
                    values = (row["x"], row["y"], row["z"])
                elif isinstance(row, dict):
                    values = (row["x"], row["y"], row["z"])
                else:
                    values = (row[0], row[1], row[2])
                point = tuple(float(value) for value in values)
            except (KeyError, IndexError, TypeError, ValueError):
                continue
            if all(math.isfinite(value) for value in point):
                points.append(point)

        # A valid empty cloud is a fresh "no obstacle" observation.  It must
        # refresh the watchdog just like a non-empty frame.
        self._obstacle_seen = True
        self._obstacle_received_at = time.monotonic()
        if len(points) < 4:
            self._obstacles = ()
            return
        cx = sum(p[0] for p in points) / len(points)
        cy = sum(p[1] for p in points) / len(points)
        cz = sum(p[2] for p in points) / len(points)
        radius = max(0.2, max(math.dist((cx, cy, cz), p) for p in points))
        self._obstacles = ((cx, cy, cz, radius),)

    def _on_quality_coverage(self, msg: String) -> None:
        """Consume camera-quality coverage produced by the perception simulator.

        The planner must not treat tower localization or a published path as
        completed inspection.  Only quality-gated observations advance the
        task state and coverage ratio.
        """
        fields = {}
        for token in str(msg.data).split():
            if "=" in token:
                key, value = token.split("=", 1)
                fields[key] = value
        try:
            ratio = max(0.0, min(1.0, float(fields.get("ratio", 0.0))))
        except (TypeError, ValueError):
            return
        self._quality_coverage_ratio = ratio
        self._quality_coverage_valid = fields.get("valid", "false").lower() == "true"
        self._quality_coverage_received = True
        self._task.observe(valid=self._quality_coverage_valid, coverage_ratio=ratio)
        if self._quality_coverage_valid and self._task.state == InspectionState.RECONNECTING:
            self._awaiting_fresh_quality = False
            self._task.resume(valid_observation=True)

    def _on_health(self, msg: String) -> None:
        """Apply simulated or vehicle-supervisor health transitions.

        The message is deliberately transport-neutral (``HEALTH
        state=STALE_POINTCLOUD action=PAUSE_AND_REPLAN``), so fault injection
        can run without coupling this package to a vendor diagnostic type.
        A non-ready health state brakes the task and suppresses publication of
        a stale trajectory.  Recovery requires a fresh observation before the
        task resumes scanning.
        """
        fields = {}
        for token in str(msg.data).split():
            if "=" in token:
                key, value = token.split("=", 1)
                fields[key.strip().lower()] = value.strip().upper()
        value = fields.get("state")
        if value is None:
            # Accept a bare enum value for simple replay scripts.
            value = str(msg.data).strip().upper()
        try:
            state = HealthState(value)
        except ValueError:
            return
        self._health_state = state
        self._health_received_at = time.monotonic()
        if state == HealthState.READY:
            if self._task.state == InspectionState.BRAKING:
                self._task.reconnect(available=True)
        else:
            self._awaiting_fresh_quality = True
            self._task.update_health(state)

    def _health_ok(self) -> bool:
        if self._health_received_at is None:
            return not bool(self.get_parameter("require_health").value)
        timeout = max(0.0, float(self.get_parameter("health_timeout").value))
        fresh = time.monotonic() - self._health_received_at <= timeout
        return fresh and self._health_state == HealthState.READY

    def _publish_empty_path(self) -> None:
        """Retract the previous reference path after a health fault."""
        path = PathMsg()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._frame_id
        self._path_pub.publish(path)

    def _path_clear_of_obstacles(self, samples, radius: float) -> bool:
        # Before any valid obstacle frame arrives there is no observation to
        # reject (this keeps offline geometry tests and startup deterministic).
        # After the first frame, however, stale data is treated as blocked
        # until a fresh frame arrives; an interrupted sensor stream must fail
        # safe rather than silently clear the route.
        obstacle_seen = bool(getattr(self, "_obstacle_seen", bool(self._obstacles)))
        if obstacle_seen:
            try:
                timeout = float(self.get_parameter("obstacle_timeout").value)
            except (AttributeError, KeyError, TypeError, ValueError):
                timeout = 0.5
            if time.monotonic() - float(self._obstacle_received_at) > max(0.0, timeout):
                return False
        margin = float(self.get_parameter("obstacle_margin").value)
        try:
            configured_step = float(self.get_parameter("obstacle_max_step").value)
        except (AttributeError, KeyError):
            configured_step = 0.25
        max_step = max(1e-3, configured_step)
        # Check the swept path, not only its published waypoints.  A moving
        # vehicle can cross a spherical obstacle between two sparse samples.
        dense = []
        samples = tuple(samples)
        for index, sample in enumerate(samples):
            point = (float(sample[0]), float(sample[1]), float(sample[2]))
            if index == 0:
                dense.append(point)
                continue
            previous = (float(samples[index - 1][0]), float(samples[index - 1][1]), float(samples[index - 1][2]))
            distance = math.dist(previous, point)
            subdivisions = max(1, int(math.ceil(distance / max_step)))
            for subdivision in range(1, subdivisions + 1):
                ratio = subdivision / subdivisions
                dense.append(tuple(previous[axis] + (point[axis] - previous[axis]) * ratio for axis in range(3)))
        for ox, oy, oz, obstacle_radius in self._obstacles:
            if any(math.dist(point, (ox, oy, oz)) <= obstacle_radius + margin for point in dense):
                return False
        return True

    def _publish_candidate(self) -> None:
        if self._scan_mode == "vertical_tower":
            self._publish_vertical_tower_path()
            return

        start_s = float(self.get_parameter("start_s").value)
        end_s = float(self.get_parameter("end_s").value)
        radii = [float(value) for value in self.get_parameter("radius_candidates").value]
        evaluations = evaluate_candidates(
            self._line,
            start_s=start_s,
            end_s=end_s,
            radii=radii,
            line_exclusion_radius=float(self.get_parameter("line_exclusion_radius").value),
            sample_dt=float(self.get_parameter("sample_dt").value),
            max_step=0.25,
            reference_radius=5.0,
        )
        selected = select_best_candidate(evaluations)
        status = String()
        if selected is None:
            self._task.initialize(has_safe_candidate=False)
            status.data = "NO_SAFE_CANDIDATE"
            self._status_pub.publish(status)
            coverage = String()
            coverage.data = "WAITING_FOR_SAFE_TRAJECTORY"
            self._coverage_pub.publish(coverage)
            return
        self._task.initialize(has_safe_candidate=True)
        if self._require_observation and not self._observation_received:
            status.data = "WAITING_FOR_VALID_OBSERVATION source=synthetic_simulation"
            self._status_pub.publish(status)
            coverage = String()
            coverage.data = "WAITING_FOR_VALID_OBSERVATIONS ratio=0.000"
            self._coverage_pub.publish(coverage)
            return
        status.data = (
            f"{self._task.state.value} radius={selected.radius:.3f} theta_amplitude_deg="
            f"{math.degrees(selected.theta_amplitude):.3f} "
            f"min_line_distance={selected.clearance.min_line_distance:.3f} "
            f"score={selected.score:.3f}"
        )
        self._status_pub.publish(status)
        coverage = String()
        coverage.data = "WAITING_FOR_VALID_OBSERVATIONS ratio=0.000"
        self._coverage_pub.publish(coverage)
        path = PathMsg()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._frame_id
        for sample in selected.trajectory:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = sample.position[0]
            pose.pose.position.y = sample.position[1]
            pose.pose.position.z = sample.position[2]
            path.poses.append(pose)
        self._path_pub.publish(path)

    def _publish_vertical_tower_path(self) -> None:
        """Plan a standoff helix from the latest PointCloud2 tower estimate."""

        status = String()
        coverage = String()
        observation = self._tower_observation
        observation_age = (
            time.monotonic() - self._tower_observation_received_at
            if self._tower_observation_received_at is not None else math.inf
        )
        timeout = float(self.get_parameter("tower_observation_timeout").value)
        if observation is None or observation_age > timeout:
            self._tower_observation = None
            self._observation_received = False
            status.data = "WAITING_FOR_VALID_TOWER_POINTCLOUD"
            coverage.data = "WAITING_FOR_TOWER_LOCALIZATION"
            self._status_pub.publish(status)
            self._coverage_pub.publish(coverage)
            return
        if not self._health_ok():
            status.data = f"WAITING_FOR_HEALTH state={self._health_state.value}"
            coverage.data = "WAITING_FOR_HEALTH_FAULT_CLEAR"
            self._status_pub.publish(status)
            self._coverage_pub.publish(coverage)
            self._publish_empty_path()
            return
        # A recovered link/sensor does not make the previous camera-quality
        # observation current again.  Keep the path retracted until a fresh
        # valid quality message arrives and explicitly resumes the task.
        if self._task.state in (InspectionState.BRAKING, InspectionState.RECONNECTING):
            status.data = "WAITING_FOR_FRESH_QUALITY_AFTER_HEALTH"
            coverage.data = "WAITING_FOR_VALID_OBSERVATIONS_AFTER_HEALTH"
            self._status_pub.publish(status)
            self._coverage_pub.publish(coverage)
            self._publish_empty_path()
            return
        try:
            plan = None
            for candidate_radius in self.get_parameter("replan_radii").value:
                candidate = plan_observed_tower_scan(
                    **observation,
                    minimum_orbit_radius=float(candidate_radius),
                    surface_standoff=float(self.get_parameter("tower_surface_standoff").value),
                    base_clearance=float(self.get_parameter("tower_base_clearance").value),
                    top_clearance=float(self.get_parameter("tower_top_clearance").value),
                    orbit_count=float(self.get_parameter("orbit_count").value),
                    samples_per_orbit=int(self.get_parameter("orbit_samples_per_turn").value),
                    start_angle=float(self.get_parameter("orbit_start_angle").value),
                )
                if self._path_clear_of_obstacles(candidate.samples, candidate.orbit_radius):
                    plan = candidate
                    break
            if plan is None:
                status.data = "NO_SAFE_TRAJECTORY_DYNAMIC_OBSTACLE"
                coverage.data = "WAITING_FOR_DYNAMIC_OBSTACLE_CLEARANCE"
                self._status_pub.publish(status)
                self._coverage_pub.publish(coverage)
                return
        except (TypeError, ValueError) as exc:
            status.data = f"INVALID_VERTICAL_TOWER_PARAMETERS reason={exc}"
            coverage.data = "WAITING_FOR_SAFE_TRAJECTORY"
            self._status_pub.publish(status)
            self._coverage_pub.publish(coverage)
            return

        # Initialize once.  Reinitializing on every timer tick would erase a
        # BRAKING/RECONNECTING transition and make fault handling ineffective.
        if self._task.state == InspectionState.INITIALIZING or not self._task.has_safe_candidate:
            self._task.initialize(has_safe_candidate=True)
        if self._require_observation and not self._observation_received:
            status.data = "WAITING_FOR_VALID_OBSERVATION source=synthetic_simulation"
            coverage.data = "WAITING_FOR_VALID_OBSERVATIONS ratio=0.000"
            self._status_pub.publish(status)
            self._coverage_pub.publish(coverage)
            return

        turns = max(2.0, float(self.get_parameter("orbit_count").value))
        previous_radius = getattr(self, "_last_plan_radius", None)
        self._last_plan_radius = float(plan.orbit_radius)
        replan_note = ""
        if previous_radius is not None and abs(previous_radius - self._last_plan_radius) > 1e-6:
            replan_note = f" REPLANNED radius={self._last_plan_radius:.3f} previous={previous_radius:.3f}"
        status.data = (
            f"VERTICAL_TOWER_READY center=({plan.center_x:.3f},{plan.center_y:.3f}) "
            f"z=({plan.z_min:.3f},{plan.z_max:.3f}) radius={plan.orbit_radius:.3f} "
            f"tower_clearance={plan.minimum_tower_clearance:.3f} "
            f"orbits={turns:.2f} points={len(plan.samples)}{replan_note}"
        )
        if getattr(self, "_quality_coverage_received", False):
            coverage.data = (
                "REFERENCE_PATH_PUBLISHED "
                f"points={len(plan.samples)} QUALITY_COVERAGE "
                f"ratio={self._task.coverage_ratio:.3f} "
                f"valid={str(getattr(self, '_quality_coverage_valid', False)).lower()}"
            )
        else:
            coverage.data = (
                "REFERENCE_PATH_PUBLISHED "
                f"points={len(plan.samples)} WAITING_FOR_VALID_OBSERVATIONS "
                f"ratio={self._task.coverage_ratio:.3f}"
            )
        self._status_pub.publish(status)
        self._coverage_pub.publish(coverage)

        path = PathMsg()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._frame_id
        for x, y, z, yaw in plan.samples:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        self._path_pub.publish(path)

    def _tower_center(self):
        """Return the configured center, optionally corrected by perception.

        The configured center keeps the simulation deterministic before the
        first observation.  Once a valid target arrives, its x/y coordinates
        can move the helix to the detected tower without changing its radius
        or altitude envelope.
        """

        center_x = float(self.get_parameter("tower_center_x").value)
        center_y = float(self.get_parameter("tower_center_y").value)
        # The perception node supplies the detected tower position in map.
        # Use it for planning once the observation gate has opened; the
        # parameters remain a deterministic fallback for replay/debugging.
        use_observation = bool(self.get_parameter("use_observation_center").value)
        if use_observation and self._observation_pose is not None:
            center_x, center_y = self._observation_pose[:2]
        return center_x, center_y


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InspectionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
