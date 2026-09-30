#!/usr/bin/env python3
"""Fuse tower point-cloud localization with camera semantic quality."""
import json
import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class PointCloudImageFusion(Node):
    def __init__(self):
        super().__init__("pointcloud_image_fusion")
        self.declare_parameter("quality_threshold", 0.6)
        self.declare_parameter("timeout_s", 1.0)
        self._threshold = float(self.get_parameter("quality_threshold").value)
        self._timeout = float(self.get_parameter("timeout_s").value)
        self._center = None
        self._center_at = 0.0
        self._vision = None
        self._vision_at = 0.0
        self._pub = self.create_publisher(PoseStamped, "/inspection/fusion/tower_pose", 10)
        self._status = self.create_publisher(String, "/inspection/fusion/status", 10)
        self.create_subscription(PoseStamped, "/inspection/tower/center", self._on_center, 10)
        self.create_subscription(String, "/inspection/vision/observation", self._on_vision, 10)
        self.create_timer(0.2, self._fuse)

    def _on_center(self, msg):
        self._center = msg
        self._center_at = time.monotonic()

    def _on_vision(self, msg):
        try:
            self._vision = json.loads(msg.data)
            self._vision_at = time.monotonic()
        except (TypeError, ValueError):
            self._vision = None

    def _fuse(self):
        now = time.monotonic()
        if self._center is None or self._vision is None:
            self._status.publish(String(data="WAITING_FOR_POINTCLOUD_AND_IMAGE"))
            return
        if now - self._center_at > self._timeout or now - self._vision_at > self._timeout:
            self._status.publish(String(data="STALE_FUSION_INPUT"))
            return
        try:
            quality = float(self._vision.get("quality", 0.0))
        except (TypeError, ValueError):
            self._status.publish(String(data="REJECTED_INVALID_QUALITY"))
            return
        if not math.isfinite(quality) or quality < 0.0 or quality > 1.0:
            self._status.publish(String(data=f"REJECTED_INVALID_QUALITY quality={quality!r}"))
            return
        # Optional metrics are emitted by the simulation detector.  Keeping
        # them optional preserves compatibility with an external detector,
        # while rejecting simulated false positives and unusable frames.
        contrast = self._vision.get("image_contrast")
        if contrast is not None:
            try:
                contrast_value = float(contrast)
            except (TypeError, ValueError):
                self._status.publish(String(data="REJECTED_IMAGE_CONTRAST"))
                return
            if not math.isfinite(contrast_value) or contrast_value < 2.0:
                self._status.publish(String(data=f"REJECTED_IMAGE_CONTRAST contrast={contrast_value:.3f}"))
                return
        depth_fraction = self._vision.get("depth_valid_fraction")
        if depth_fraction is not None:
            try:
                depth_value = float(depth_fraction)
            except (TypeError, ValueError):
                self._status.publish(String(data="REJECTED_DEPTH_SPARSE"))
                return
            if not math.isfinite(depth_value) or depth_value < 0.01:
                self._status.publish(String(data=f"REJECTED_DEPTH_SPARSE fraction={depth_value:.4f}"))
                return
        confidence = self._vision.get("detection_confidence")
        if confidence is not None:
            try:
                confidence_value = float(confidence)
            except (TypeError, ValueError):
                self._status.publish(String(data="REJECTED_DETECTION_CONFIDENCE"))
                return
            if not math.isfinite(confidence_value) or confidence_value < self._threshold:
                self._status.publish(String(data=f"REJECTED_DETECTION_CONFIDENCE confidence={confidence_value:.3f}"))
                return
        false_positive = self._vision.get("false_positive", False)
        if isinstance(false_positive, str):
            false_positive = false_positive.strip().lower() in ("1", "true", "yes", "on")
        if bool(false_positive):
            self._status.publish(String(data="REJECTED_FALSE_POSITIVE"))
            return
        if not self._vision.get("depth_ready", False) or not self._vision.get("tower", False) or quality < self._threshold:
            self._status.publish(String(data=f"REJECTED_IMAGE_QUALITY quality={quality:.3f}"))
            return
        out = PoseStamped()
        out.header = self._center.header
        out.pose = self._center.pose
        # A small deterministic covariance proxy: image quality scales the
        # point-cloud uncertainty represented by the positional footprint.
        out.pose.position.x += 0.01 * (1.0 - quality)
        out.pose.position.y += 0.01 * (1.0 - quality)
        out.pose.position.z += 0.005 * (1.0 - quality)
        self._pub.publish(out)
        self._status.publish(String(data=f"FUSED_VALID frame={out.header.frame_id} quality={quality:.3f} pair_dt={abs(now - self._center_at):.3f}s"))


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudImageFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
