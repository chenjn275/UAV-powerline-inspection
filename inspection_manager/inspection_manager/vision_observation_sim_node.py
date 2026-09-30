#!/usr/bin/env python3
"""Deterministic camera quality and semantic-observation simulation.

Consumes the simulated D435i RGB/depth streams and emits a compact JSON-like
String record.  The record is intentionally transport-neutral so it can be
replaced by a detector without changing the manager contract.
"""
import json
import math
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class VisionObservationSim(Node):
    def __init__(self):
        super().__init__("vision_observation_sim")
        self.declare_parameter("quality_cycle_s", 12.0)
        self.declare_parameter("min_depth_bytes", 100)
        self._cycle = max(1.0, float(self.get_parameter("quality_cycle_s").value))
        self._min_depth_bytes = int(self.get_parameter("min_depth_bytes").value)
        self._rgb = None
        self._depth = None
        self._started = self.get_clock().now().nanoseconds / 1e9
        self._sequence = 0
        self._pub = self.create_publisher(String, "/inspection/vision/observation", 10)
        self.create_subscription(Image, "/inspection_d435i/color/image_raw", self._on_rgb, 10)
        self.create_subscription(Image, "/inspection_d435i/depth/image_rect_raw", self._on_depth, 10)
        self.create_timer(0.2, self._publish)

    @staticmethod
    def _fault_profile(phase, cycle):
        """Return deterministic quality/fault values for one camera view."""
        if phase < cycle * 0.45:
            return 0.92, 0.05, 0.03, False
        if phase < cycle * 0.65:
            return 0.42, 0.65, 0.35, False
        if phase < cycle * 0.85:
            return 0.68, 0.25, 0.18, False
        # A plausible-looking frame with a deliberate semantic false positive.
        # The confidence penalty and explicit flag make downstream fusion and
        # coverage reject it even though image transport is healthy.
        return 0.72, 0.08, 0.02, True

    @staticmethod
    def _depth_ready(msg, minimum_bytes=100):
        """Require a structurally valid image with at least one valid depth.

        A zero-filled depth frame has the right byte count but carries no
        observation.  Sampling the payload keeps this check cheap while
        making the simulated rejection path match a real sensor timeout.
        """
        if msg is None or len(msg.data) < minimum_bytes:
            return False
        if getattr(msg, "encoding", "") not in ("16UC1", "mono16"):
            return False
        if getattr(msg, "step", 0) < max(2, int(getattr(msg, "width", 0)) * 2):
            return False
        sample_count = len(msg.data) // 2
        payload = memoryview(msg.data)
        # Check a bounded, evenly spaced sample so the middle tower silhouette
        # is observed without scanning a full VGA frame every timer tick.
        stride = max(1, sample_count // 512)
        for index in range(0, sample_count, stride):
            if struct.unpack_from("<H", payload, index * 2)[0] > 0:
                return True
        return False

    @staticmethod
    def _depth_valid_fraction(msg):
        if msg is None or len(msg.data) < 2:
            return 0.0
        payload = memoryview(msg.data)
        count = len(msg.data) // 2
        valid = 0
        for index in range(count):
            valid += struct.unpack_from("<H", payload, index * 2)[0] > 0
        return valid / float(count)

    @staticmethod
    def _rgb_contrast(msg):
        if msg is None or len(msg.data) < 6:
            return 0.0
        payload = memoryview(msg.data)
        stride = max(3, len(msg.data) // (3 * 512))
        values = []
        for index in range(0, len(msg.data) - 2, stride):
            values.append((int(payload[index]) + int(payload[index + 1]) + int(payload[index + 2])) / 3.0)
        if not values:
            return 0.0
        mean = sum(values) / len(values)
        return math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))

    def _on_rgb(self, msg):
        self._rgb = msg

    def _on_depth(self, msg):
        self._depth = msg

    def _publish(self):
        now = self.get_clock().now().nanoseconds / 1e9
        phase = (now - self._started) % self._cycle
        quality, occlusion, miss, false_positive = self._fault_profile(phase, self._cycle)
        rgb_ready = (
            self._rgb is not None
            and getattr(self._rgb, "encoding", "") in ("rgb8", "bgr8")
            and len(self._rgb.data) >= max(3, int(getattr(self._rgb, "width", 0)) * 3)
        )
        depth_fraction = self._depth_valid_fraction(self._depth)
        contrast = self._rgb_contrast(self._rgb)
        ready = rgb_ready and self._depth_ready(self._depth, self._min_depth_bytes) and depth_fraction >= 0.01
        visual_ready = ready
        if visual_ready and contrast < 2.0:
            # A flat/overexposed frame is transport-ready but visually unusable.
            quality = min(quality, 0.35)
            visual_ready = False
        detected = visual_ready and quality > 0.5
        confidence_penalty = 0.2 if false_positive else 1.0
        payload = {
            "source": "d435i_sim",
            # A monotonically increasing view id represents the vehicle's
            # progress along the reference path.  Coverage uses it to avoid
            # counting repeated samples of one camera pose as new cells.
            "view_id": self._sequence,
            "tower": detected,
            "conductors": detected,
            "insulators": detected and quality > 0.75,
            "semantic_truth": not false_positive,
            "false_positive": false_positive,
            "missed_detection": bool(ready and not detected and not false_positive),
            "quality": round(quality if ready else 0.0, 3),
            "occlusion": round(occlusion, 3),
            "miss_probability": round(miss, 3),
            "false_positive_probability": round(0.02 + (1.0 - quality) * 0.12, 3),
            "detection_confidence": round(max(0.0, confidence_penalty * quality * (1.0 - occlusion) * (1.0 - miss)), 3),
            "depth_ready": ready,
            "depth_valid_fraction": round(depth_fraction, 4),
            "image_contrast": round(contrast, 3),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self._pub.publish(msg)
        self._sequence += 1


def main(args=None):
    rclpy.init(args=args)
    node = VisionObservationSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
