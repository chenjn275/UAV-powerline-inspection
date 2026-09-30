import math
import unittest

from inspection_core.coverage import CoverageGrid, CoverageObservation
from inspection_core.candidate_search import evaluate_candidates, select_best_candidate, validate_swept_clearance
from inspection_core.health import HealthSnapshot, HealthState
from inspection_core.state_machine import InspectionState, InspectionTask
from inspection_core.coordinate import LocalAlignment
from inspection_core.line_model import (
    ArcScanSpec,
    PolylineLineModel,
    SphereObstacle,
    generate_arc_scan,
    validate_clearance,
)


class GeometryTests(unittest.TestCase):
    def test_straight_line_has_stable_frame_and_distance(self):
        line = PolylineLineModel([(0, 0, 0), (10, 0, 0)], model_version="straight-v1")
        sample = line.sample(5.0)
        self.assertAlmostEqual(line.length, 10.0)
        self.assertAlmostEqual(line.nearest_distance((5, 3, 4)), 5.0)
        self.assertAlmostEqual(sum(v * v for v in sample.normal), 1.0)
        self.assertAlmostEqual(sum(v * v for v in sample.binormal), 1.0)
        self.assertAlmostEqual(sum(a * b for a, b in zip(sample.tangent, sample.normal)), 0.0)

    def test_parallel_transport_survives_zero_curvature_segment(self):
        line = PolylineLineModel([(0, 0, 0), (5, 0, 0), (10, 0, 0), (10, 5, 0)])
        frames = [line.sample(s) for s in (0, 2.5, 5, 7.5, 10, 12.5, 15)]
        for frame in frames:
            self.assertTrue(all(math.isfinite(value) for value in frame.normal + frame.binormal))
            self.assertAlmostEqual(sum(a * b for a, b in zip(frame.normal, frame.binormal)), 0.0, places=6)

    def test_arc_scan_keeps_cross_section_radius(self):
        line = PolylineLineModel([(0, 0, 0), (10, 0, 0)])
        samples = generate_arc_scan(
            line,
            ArcScanSpec(start_s=0.0, end_s=10.0, radius=5.0, theta_amplitude=math.radians(10), sample_dt=0.1),
        )
        self.assertGreater(len(samples), 2)
        self.assertAlmostEqual(samples[0].s, 0.0)
        self.assertAlmostEqual(samples[-1].s, 10.0)
        self.assertTrue(all(a.s <= b.s for a, b in zip(samples, samples[1:])))
        self.assertTrue(all(abs(line.nearest_distance(sample.position) - 5.0) < 1e-8 for sample in samples))

    def test_clearance_checks_full_line_and_obstacles(self):
        line = PolylineLineModel([(0, 0, 0), (10, 0, 0)])
        safe = [(5, 5, 0)]
        self.assertTrue(validate_clearance(line, safe, line_exclusion_radius=2.0).accepted)
        obstacle = SphereObstacle(center=(5, 5, 0), radius=1.0, name="tower")
        report = validate_clearance(line, safe, line_exclusion_radius=2.0, obstacles=[obstacle], obstacle_margin=2.0)
        self.assertFalse(report.accepted)
        self.assertEqual(report.violation_reason, "obstacle:tower")

    def test_swept_check_catches_between_sample_obstacle(self):
        line = PolylineLineModel([(0, 0, 0), (10, 0, 0)])
        samples = generate_arc_scan(line, ArcScanSpec(start_s=0.0, end_s=10.0, radius=5.0, sample_dt=10.0))
        obstacle = SphereObstacle(center=(5, -0.6, -4.96), radius=0.5, name="midpoint")
        report = validate_swept_clearance(line, samples, line_exclusion_radius=2.0, obstacles=[obstacle], obstacle_margin=1.0, max_step=0.1)
        self.assertFalse(report.accepted)
        self.assertEqual(report.violation_reason, "obstacle:midpoint")

    def test_candidate_search_rejects_unsafe_radius(self):
        line = PolylineLineModel([(0, 0, 0), (10, 0, 0)])
        obstacle = SphereObstacle(center=(5, 0, -2.8), radius=0.5, name="tower")
        evaluations = evaluate_candidates(line, start_s=0, end_s=10, radii=[4.0, 5.0], line_exclusion_radius=2.0, obstacles=[obstacle], obstacle_margin=1.0, sample_dt=0.2, max_step=0.1, reference_radius=5.0)
        best = select_best_candidate(evaluations)
        self.assertIsNotNone(best)
        self.assertEqual(best.radius, 5.0)
        self.assertTrue(any(not evaluation.accepted for evaluation in evaluations))


class CoverageTests(unittest.TestCase):
    def test_quality_gates_and_reconnect(self):
        grid = CoverageGrid(
            s_min=0,
            s_max=10,
            s_bins=2,
            theta_min=-1,
            theta_max=1,
            theta_bins=2,
            min_distance=4,
            max_distance=6,
            min_sharpness=0.8,
        )
        self.assertFalse(
            grid.mark(CoverageObservation(s=1, theta=-0.5, distance=5, visible=True, clear=True, sharpness=0.2))
        )
        self.assertTrue(
            grid.mark(CoverageObservation(s=1, theta=-0.5, distance=5, visible=True, clear=True, sharpness=0.9))
        )
        self.assertAlmostEqual(grid.coverage_ratio, 0.25)
        reconnect = grid.choose_reconnect(current_s=1, current_theta=-0.5)
        self.assertIsNotNone(reconnect)
        self.assertTrue(grid.choose_reconnect(current_s=1, current_theta=-0.5, reachable=lambda s, _: s > 20) is None)


class HealthTests(unittest.TestCase):
    def _snapshot(self, **changes):
        values = dict(
            now=10.0, pointcloud_stamp=9.8, trajectory_stamp=9.9,
            model_stamp=9.0, link_last_seen=9.9,
            pointcloud_timeout=1.0, trajectory_timeout=0.5,
            model_timeout=5.0, link_timeout=1.0,
        )
        values.update(changes)
        return HealthSnapshot(**values)

    def test_ready_and_stale_trajectory_actions(self):
        self.assertEqual(self._snapshot().state, HealthState.READY)
        stale = self._snapshot(trajectory_stamp=8.0)
        self.assertEqual(stale.state, HealthState.STALE_TRAJECTORY)
        self.assertEqual(stale.control_action, "BRAKE_OR_VERIFIED_BACKUP")

    def test_multiple_faults_are_not_promoted_to_rtl(self):
        failed = self._snapshot(pointcloud_stamp=None, link_last_seen=None)
        self.assertEqual(failed.state, HealthState.MULTIPLE_FAULTS)
        self.assertEqual(failed.control_action, "BRAKE_AND_REQUIRE_SUPERVISOR_DECISION")


class StateMachineTests(unittest.TestCase):
    def test_scan_fault_reconnect_and_completion_gate(self):
        task = InspectionTask()
        self.assertEqual(task.initialize(has_safe_candidate=True), InspectionState.WAITING_FOR_OBSERVATION)
        self.assertEqual(task.observe(valid=True, coverage_ratio=0.4), InspectionState.SCANNING)
        self.assertEqual(task.update_health(HealthState.STALE_TRAJECTORY), InspectionState.BRAKING)
        self.assertEqual(task.reconnect(available=True), InspectionState.RECONNECTING)
        self.assertEqual(task.resume(valid_observation=True), InspectionState.SCANNING)
        self.assertEqual(task.finish(), InspectionState.SCANNING)
        task.coverage_ratio = 0.95
        self.assertEqual(task.finish(), InspectionState.COMPLETE)


class CoordinateTests(unittest.TestCase):
    def test_enu_to_ned_axis_and_origin(self):
        alignment = LocalAlignment(origin_ned=(10, 20, 30))
        self.assertEqual(alignment.position_enu_to_ned((0, 0, 0)), (10, 20, 30))
        self.assertEqual(alignment.position_enu_to_ned((1, 0, 0)), (10, 21, 30))
        self.assertEqual(alignment.position_enu_to_ned((0, 1, 0)), (11, 20, 30))
        self.assertEqual(alignment.position_enu_to_ned((0, 0, 2)), (10, 20, 28))

    def test_yaw_offset_is_explicit(self):
        alignment = LocalAlignment(yaw_offset=math.pi / 2)
        self.assertTrue(alignment.validate_round_trip((1, 0, 0), (-1, 0, 0)))


if __name__ == "__main__":
    unittest.main()
