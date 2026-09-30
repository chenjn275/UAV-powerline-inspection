#!/usr/bin/env python3
"""Regression checks for the simulation camera/fusion/coverage contract."""

import json
import struct
import unittest
import time

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String

from inspection_manager.quality_coverage_sim_node import QualityCoverageSim
from inspection_manager.vision_observation_sim_node import VisionObservationSim
from inspection_manager.pointcloud_image_fusion_sim_node import PointCloudImageFusion


class _Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class PerceptionPipelineTest(unittest.TestCase):
    def test_zero_depth_frame_is_not_ready(self):
        msg = Image(width=320, height=240, encoding="16UC1", step=640)
        msg.data = bytes(320 * 240 * 2)
        self.assertFalse(VisionObservationSim._depth_ready(msg))
        self.assertEqual(VisionObservationSim._depth_valid_fraction(msg), 0.0)

    def test_fault_profile_covers_occlusion_miss_and_false_positive(self):
        cycle = 12.0
        clear = VisionObservationSim._fault_profile(1.0, cycle)
        occluded = VisionObservationSim._fault_profile(6.0, cycle)
        degraded = VisionObservationSim._fault_profile(8.0, cycle)
        false_positive = VisionObservationSim._fault_profile(11.0, cycle)
        self.assertFalse(clear[3])
        self.assertLess(occluded[0], 0.6)
        self.assertGreater(occluded[1], 0.5)
        self.assertFalse(degraded[3])
        self.assertTrue(false_positive[3])
        self.assertGreater(false_positive[0], 0.6)

    def test_sparse_valid_depth_and_image_contrast_are_detectable(self):
        depth = Image(width=32, height=24, encoding="16UC1", step=64)
        payload = bytearray(32 * 24 * 2)
        struct.pack_into("<H", payload, (12 * 32 + 16) * 2, 6000)
        depth.data = bytes(payload)
        self.assertTrue(VisionObservationSim._depth_ready(depth))
        self.assertGreater(VisionObservationSim._depth_valid_fraction(depth), 0.0)

        flat = Image(width=4, height=2, encoding="rgb8", step=12)
        flat.data = bytes([40, 40, 40] * 8)
        self.assertAlmostEqual(VisionObservationSim._rgb_contrast(flat), 0.0)
        varied = Image(width=4, height=2, encoding="rgb8", step=12)
        varied.data = bytes([0, 0, 0, 255, 255, 255] * 4)
        self.assertGreater(VisionObservationSim._rgb_contrast(varied), 2.0)

    def test_quality_coverage_rejects_low_detection_confidence(self):
        node = QualityCoverageSim.__new__(QualityCoverageSim)
        node._cells = 10
        node._threshold = 0.6
        node._completed = 0
        node._last_key = None
        node._pub = _Capture()
        node._on_observation(String(data=json.dumps({
            "tower": True,
            "conductors": True,
            "depth_ready": True,
            "quality": 0.92,
            "detection_confidence": 0.45,
        })))
        self.assertEqual(node._completed, 0)
        self.assertIn("valid=false", node._pub.messages[-1].data)

        node._on_observation(String(data=json.dumps({
            "tower": True,
            "conductors": True,
            "depth_ready": True,
            "quality": 0.92,
            "detection_confidence": 0.90,
            "view_id": 10,
        })))
        self.assertEqual(node._completed, 1)
        self.assertIn("valid=true", node._pub.messages[-1].data)
        # Repeated data from one camera pose does not over-count coverage.
        node._on_observation(String(data=json.dumps({
            "tower": True, "conductors": True, "depth_ready": True,
            "quality": 0.92, "detection_confidence": 0.90, "view_id": 10,
        })))
        self.assertEqual(node._completed, 1)
        node._on_observation(String(data=json.dumps({
            "tower": True, "conductors": True, "depth_ready": True,
            "quality": 0.92, "detection_confidence": 0.90, "view_id": 11,
        })))
        self.assertEqual(node._completed, 2)

        node._on_observation(String(data=json.dumps({
            "tower": True, "conductors": True, "depth_ready": True,
            "quality": 0.9, "detection_confidence": 0.9,
            "false_positive": True, "view_id": 12,
        })))
        self.assertEqual(node._completed, 2)
        self.assertIn("valid=false", node._pub.messages[-1].data)

    def test_quality_coverage_parses_string_booleans_and_ignores_non_objects(self):
        node = QualityCoverageSim.__new__(QualityCoverageSim)
        node._cells = 4
        node._threshold = 0.6
        node._completed = 0
        node._last_key = None
        node._pub = _Capture()
        node._on_observation(String(data="[]"))
        self.assertFalse(node._pub.messages)
        node._on_observation(String(data=json.dumps({
            "tower": "true", "conductors": "true", "depth_ready": "true",
            "quality": "0.9", "detection_confidence": "0.8", "view_id": 1,
        })))
        self.assertEqual(node._completed, 1)
        self.assertIn("valid=true", node._pub.messages[-1].data)

    def test_fusion_rejects_bad_image_and_publishes_quality_gated_pose(self):
        node = PointCloudImageFusion.__new__(PointCloudImageFusion)
        node._threshold = 0.6
        node._timeout = 1.0
        node._center_at = time.monotonic()
        node._vision_at = node._center_at
        node._center = PoseStamped()
        node._center.header.frame_id = "map"
        node._center.pose.position.x = 2.0
        node._center.pose.position.y = -1.0
        node._center.pose.position.z = 4.0
        node._vision = {
            "quality": 0.9,
            "tower": True,
            "depth_ready": True,
            "image_contrast": 0.1,
            "depth_valid_fraction": 0.08,
            "detection_confidence": 0.8,
        }
        node._pub = _Capture()
        node._status = _Capture()
        node._fuse()
        self.assertFalse(node._pub.messages)
        self.assertIn("REJECTED_IMAGE_CONTRAST", node._status.messages[-1].data)

        node._vision["image_contrast"] = 12.0
        node._fuse()
        self.assertEqual(len(node._pub.messages), 1)
        self.assertIn("FUSED_VALID", node._status.messages[-1].data)
        self.assertAlmostEqual(node._pub.messages[0].pose.position.x, 2.001)

        node._vision["false_positive"] = True
        node._fuse()
        self.assertEqual(len(node._pub.messages), 1)
        self.assertIn("REJECTED_FALSE_POSITIVE", node._status.messages[-1].data)


if __name__ == "__main__":
    unittest.main()
