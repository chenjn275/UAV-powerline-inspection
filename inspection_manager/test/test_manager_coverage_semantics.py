#!/usr/bin/env python3
"""Regression test: tower localization is not image coverage."""

import sys
import time
import unittest
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import String

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parent
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "inspection_core"))

from inspection_core import HealthState, InspectionState, InspectionTask  # noqa: E402
from inspection_manager import inspection_manager_node as manager_module  # noqa: E402
from inspection_manager.obstacle_pointcloud_sim_node import moving_obstacle_center  # noqa: E402


class PublisherCapture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakePath:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.poses = []


class FakePoseStamped:
    def __init__(self):
        self.header = None
        self.pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(z=0.0, w=1.0),
        )


class ManagerCoverageSemanticsTest(unittest.TestCase):
    def test_dynamic_obstacle_trajectory_is_deterministic_and_finite(self):
        start = moving_obstacle_center(0.0)
        quarter = moving_obstacle_center(3.141592653589793)
        half = moving_obstacle_center(2.0 * 3.141592653589793)
        self.assertEqual(start, (4.0, 0.0, 4.0))
        self.assertAlmostEqual(quarter[0], 7.0)
        self.assertAlmostEqual(half[0], 4.0)
        self.assertEqual(quarter[1:], (0.0, 4.0))
        with self.assertRaises(ValueError):
            moving_obstacle_center(float("nan"))

    def test_dynamic_obstacle_clearance_rejects_and_releases_path(self):
        manager = manager_module.InspectionManager.__new__(manager_module.InspectionManager)
        manager._obstacle_received_at = time.monotonic()
        manager._obstacles = ((0.0, 0.0, 2.0, 0.5),)
        values = {"obstacle_timeout": 1.0, "obstacle_margin": 0.5}
        manager.get_parameter = lambda name: SimpleNamespace(value=values[name])
        blocked = [(0.0, 0.0, 2.0, 0.0)]
        clear = [(3.0, 0.0, 2.0, 0.0)]
        self.assertFalse(manager._path_clear_of_obstacles(blocked, 3.0))
        self.assertTrue(manager._path_clear_of_obstacles(clear, 3.0))

    def test_stale_obstacle_stream_fails_safe_after_first_valid_frame(self):
        manager = manager_module.InspectionManager.__new__(manager_module.InspectionManager)
        manager._obstacle_seen = True
        manager._obstacle_received_at = time.monotonic() - 2.0
        manager._obstacles = ()
        manager.get_parameter = lambda name: SimpleNamespace(value=0.5)
        self.assertFalse(manager._path_clear_of_obstacles([(10.0, 0.0, 2.0, 0.0)], 3.0))

    def test_obstacle_parser_accepts_generator_rows_and_rejects_wrong_frame(self):
        manager = manager_module.InspectionManager.__new__(manager_module.InspectionManager)
        manager._frame_id = "map"
        manager._obstacles = ()
        manager._obstacle_seen = False
        manager._obstacle_received_at = 0.0
        fields = [SimpleNamespace(name=name) for name in ("x", "y", "z")]
        msg = SimpleNamespace(header=SimpleNamespace(frame_id="map"), fields=fields)
        rows = ((1.0, 2.0, 3.0), (1.2, 2.0, 3.0), (0.8, 2.0, 3.0), (1.0, 2.2, 3.0))
        with patch.object(manager_module.point_cloud2, "read_points", return_value=iter(rows)):
            manager._on_obstacles(msg)
        self.assertTrue(manager._obstacle_seen)
        self.assertEqual(len(manager._obstacles), 1)
        cx, cy, cz, radius = manager._obstacles[0]
        self.assertAlmostEqual(cx, 1.0)
        self.assertAlmostEqual(cy, 2.05)
        self.assertAlmostEqual(cz, 3.0)
        self.assertGreaterEqual(radius, 0.2)

        manager._obstacle_seen = False
        manager._obstacle_received_at = 0.0
        wrong_frame = SimpleNamespace(header=SimpleNamespace(frame_id="odom"), fields=fields)
        with patch.object(manager_module.point_cloud2, "read_points", return_value=iter(rows)):
            manager._on_obstacles(wrong_frame)
        self.assertFalse(manager._obstacle_seen)

    def test_empty_cloud_refreshes_sensor_watchdog(self):
        manager = manager_module.InspectionManager.__new__(manager_module.InspectionManager)
        manager._frame_id = "map"
        manager._obstacles = ((1.0, 2.0, 3.0, 1.0),)
        manager._obstacle_seen = False
        manager._obstacle_received_at = 0.0
        manager.get_parameter = lambda name: SimpleNamespace(value={
            "obstacle_timeout": 0.5,
            "obstacle_margin": 0.5,
            "obstacle_max_step": 0.25,
        }[name])
        fields = [SimpleNamespace(name=name) for name in ("x", "y", "z")]
        msg = SimpleNamespace(header=SimpleNamespace(frame_id="map"), fields=fields)
        with patch.object(manager_module.point_cloud2, "read_points", return_value=iter(())):
            manager._on_obstacles(msg)
        self.assertTrue(manager._obstacle_seen)
        self.assertEqual(manager._obstacles, ())
        self.assertTrue(manager._path_clear_of_obstacles([(10.0, 0.0, 2.0, 0.0)], 3.0))

    def test_dynamic_obstacle_swept_check_catches_between_waypoints(self):
        manager = manager_module.InspectionManager.__new__(manager_module.InspectionManager)
        manager._obstacle_received_at = time.monotonic()
        manager._obstacles = ((5.0, 0.0, 2.0, 0.5),)
        values = {"obstacle_timeout": 1.0, "obstacle_margin": 0.5, "obstacle_max_step": 0.25}
        manager.get_parameter = lambda name: SimpleNamespace(value=values[name])
        # Endpoints are clear; the segment passes through the obstacle.
        path = [(0.0, 0.0, 2.0, 0.0), (10.0, 0.0, 2.0, 0.0)]
        self.assertFalse(manager._path_clear_of_obstacles(path, 3.0))

    def test_pointcloud_outputs_are_paired_and_converted_to_bounds(self):
        manager = manager_module.InspectionManager.__new__(manager_module.InspectionManager)
        manager._frame_id = "map"
        manager._tower_center_msg = None
        manager._tower_extent_msg = None
        manager._tower_observation = None
        manager._observation_received = False
        manager._observation_pose = None

        center = PoseStamped()
        center.header.frame_id = "map"
        center.header.stamp.sec = 7
        center.pose.position.x = 4.0
        center.pose.position.y = -1.0
        center.pose.position.z = 6.05
        extent = Vector3Stamped()
        extent.header = center.header
        extent.vector.x = 11.4
        extent.vector.y = 1.23
        extent.vector.z = 153.0

        manager._on_tower_center(center)
        self.assertIsNone(manager._tower_observation)
        manager._on_tower_extent(extent)
        self.assertAlmostEqual(manager._tower_observation["base_z"], 0.35)
        self.assertAlmostEqual(manager._tower_observation["top_z"], 11.75)
        self.assertAlmostEqual(manager._tower_observation["footprint_radius"], 1.23)
        self.assertTrue(manager._observation_received)

        manager._on_tower_status(String(data="REJECTED: sparse cloud"))
        self.assertIsNone(manager._tower_observation)
        self.assertFalse(manager._observation_received)

    def test_tower_localization_publishes_path_without_faking_coverage(self):
        manager = manager_module.InspectionManager.__new__(manager_module.InspectionManager)
        manager._frame_id = "map"
        manager._task = InspectionTask()
        manager._task.initialize(has_safe_candidate=True)
        manager._health_received_at = None
        manager._health_state = manager_module.HealthState.READY
        manager._require_observation = True
        manager._observation_received = True
        manager._observation_pose = (1.25, -2.5, 4.0)
        manager._tower_observation = {
            "center_x": 1.25,
            "center_y": -2.5,
            "base_z": 0.0,
            "top_z": 12.0,
            "footprint_radius": 0.3,
        }
        manager._tower_observation_received_at = time.monotonic()
        manager._obstacle_received_at = 0.0
        manager._obstacles = ()
        manager._status_pub = PublisherCapture()
        manager._coverage_pub = PublisherCapture()
        manager._path_pub = PublisherCapture()
        manager.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: "test-time")
        )
        values = {
            "tower_center_x": 0.0,
            "tower_center_y": 0.0,
            "use_observation_center": True,
            "z_min": 2.0,
            "z_max": 9.5,
            "orbit_radius": 4.0,
            "orbit_count": 2.0,
            "orbit_samples_per_turn": 36,
            "orbit_start_angle": 0.0,
            "minimum_orbit_radius": 3.0,
            "tower_surface_standoff": 2.0,
            "tower_base_clearance": 1.5,
            "tower_top_clearance": 1.0,
            "tower_observation_timeout": 2.0,
            "replan_radii": [3.0, 4.0, 5.0, 6.0],
            "obstacle_timeout": 0.5,
            "obstacle_margin": 1.0,
            "require_health": False,
            "health_timeout": 1.0,
        }
        manager.get_parameter = lambda name: SimpleNamespace(value=values[name])

        with patch.object(manager_module, "PathMsg", FakePath), patch.object(
            manager_module, "PoseStamped", FakePoseStamped
        ):
            manager._publish_vertical_tower_path()

        self.assertEqual(len(manager._path_pub.messages), 1)
        path = manager._path_pub.messages[0]
        self.assertEqual(path.header.frame_id, "map")
        self.assertGreater(len(path.poses), 1)
        self.assertAlmostEqual(path.poses[0].pose.position.x, 4.25)
        self.assertAlmostEqual(path.poses[0].pose.position.y, -2.5)
        self.assertAlmostEqual(path.poses[0].pose.position.z, 1.5)

        coverage = manager._coverage_pub.messages[-1].data
        self.assertIn("REFERENCE_PATH_PUBLISHED", coverage)
        self.assertIn("WAITING_FOR_VALID_OBSERVATIONS", coverage)
        self.assertIn("ratio=0.000", coverage)
        self.assertEqual(manager._task.coverage_ratio, 0.0)
        self.assertEqual(manager._task.state, InspectionState.WAITING_FOR_OBSERVATION)

    def test_quality_coverage_advances_task_only_when_valid(self):
        manager = manager_module.InspectionManager.__new__(manager_module.InspectionManager)
        manager._task = InspectionTask()
        manager._task.initialize(has_safe_candidate=True)
        manager._quality_coverage_ratio = 0.0
        manager._quality_coverage_valid = False
        manager._quality_coverage_received = False

        manager._on_quality_coverage(String(
            data="QUALITY_COVERAGE completed=0/20 ratio=0.000 valid=false quality=0.420"
        ))
        self.assertEqual(manager._task.state, InspectionState.WAITING_FOR_OBSERVATION)
        self.assertEqual(manager._task.coverage_ratio, 0.0)

        manager._on_quality_coverage(String(
            data="QUALITY_COVERAGE completed=4/20 ratio=0.200 valid=true quality=0.920"
        ))
        self.assertEqual(manager._task.state, InspectionState.SCANNING)
        self.assertAlmostEqual(manager._task.coverage_ratio, 0.2)
        self.assertTrue(manager._quality_coverage_received)

    def test_health_fault_brakes_and_fresh_quality_reconnects(self):
        manager = manager_module.InspectionManager.__new__(manager_module.InspectionManager)
        manager._task = InspectionTask()
        manager._task.initialize(has_safe_candidate=True)
        manager._health_state = HealthState.READY
        manager._health_received_at = None
        manager._quality_coverage_valid = False
        manager._quality_coverage_received = False
        manager.get_parameter = lambda name: SimpleNamespace(value={
            "require_health": True,
            "health_timeout": 1.0,
        }[name])

        manager._on_health(String(data="HEALTH state=STALE_POINTCLOUD action=PAUSE_AND_REPLAN"))
        self.assertEqual(manager._health_state, HealthState.STALE_POINTCLOUD)
        self.assertEqual(manager._task.state, InspectionState.BRAKING)
        self.assertFalse(manager._health_ok())

        manager._on_health(String(data="HEALTH state=READY action=ALLOW_CURRENT_TRAJECTORY"))
        # Recovery without a valid camera observation remains conservative.
        self.assertEqual(manager._task.state, InspectionState.RECONNECTING)
        manager._on_quality_coverage(String(
            data="QUALITY_COVERAGE completed=3/20 ratio=0.150 valid=true quality=0.920"
        ))
        self.assertEqual(manager._task.state, InspectionState.SCANNING)
        self.assertTrue(manager._health_ok())

    def test_health_recovery_does_not_reuse_quality_from_before_fault(self):
        manager = manager_module.InspectionManager.__new__(manager_module.InspectionManager)
        manager._task = InspectionTask()
        manager._task.initialize(has_safe_candidate=True)
        manager._health_state = HealthState.READY
        manager._health_received_at = None
        manager._quality_coverage_valid = False
        manager._quality_coverage_received = False
        manager._awaiting_fresh_quality = False
        manager.get_parameter = lambda name: SimpleNamespace(value={
            "require_health": True,
            "health_timeout": 1.0,
        }[name])

        # A valid sample before the fault is enough to enter scanning.
        manager._on_quality_coverage(String(
            data="QUALITY_COVERAGE completed=4/20 ratio=0.200 valid=true quality=0.920"
        ))
        self.assertEqual(manager._task.state, InspectionState.SCANNING)

        manager._on_health(String(data="HEALTH state=LINK_LOST action=HOLD_ONLY_IF_INDEPENDENT_ESTIMATE_VALID"))
        self.assertEqual(manager._task.state, InspectionState.BRAKING)
        manager._on_health(String(data="HEALTH state=READY action=ALLOW_CURRENT_TRAJECTORY"))
        self.assertEqual(manager._task.state, InspectionState.RECONNECTING)

        # The old valid sample must not reopen the task; a new valid frame is
        # required after reconnecting.
        manager._on_quality_coverage(String(
            data="QUALITY_COVERAGE completed=4/20 ratio=0.200 valid=false quality=0.410"
        ))
        self.assertEqual(manager._task.state, InspectionState.RECONNECTING)
        manager._on_quality_coverage(String(
            data="QUALITY_COVERAGE completed=5/20 ratio=0.250 valid=true quality=0.930"
        ))
        self.assertEqual(manager._task.state, InspectionState.SCANNING)

    def test_health_fault_parser_rejects_unknown_state(self):
        manager = manager_module.InspectionManager.__new__(manager_module.InspectionManager)
        manager._task = InspectionTask()
        manager._task.initialize(has_safe_candidate=True)
        manager._health_state = HealthState.READY
        manager._health_received_at = None
        manager._on_health(String(data="HEALTH state=NOT_A_REAL_STATE"))
        self.assertEqual(manager._health_state, HealthState.READY)
        self.assertIsNone(manager._health_received_at)


if __name__ == "__main__":
    unittest.main()
