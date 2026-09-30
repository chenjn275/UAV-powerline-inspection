#!/usr/bin/env python3
"""Drive simulated coverage from camera observation quality."""
import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class QualityCoverageSim(Node):
    def __init__(self):
        super().__init__("quality_coverage_sim")
        self.declare_parameter("cells", 20)
        self.declare_parameter("quality_threshold", 0.6)
        self._cells = max(1, int(self.get_parameter("cells").value))
        self._threshold = float(self.get_parameter("quality_threshold").value)
        self._completed = 0
        self._last_key = None
        self._pub = self.create_publisher(String, "/inspection/coverage_quality", 10)
        self.create_subscription(String, "/inspection/vision/observation", self._on_observation, 10)

    def _on_observation(self, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        try:
            quality = float(data.get("quality", 0.0))
            confidence = float(data.get("detection_confidence", quality))
        except (TypeError, ValueError):
            quality, confidence = 0.0, 0.0
        def as_bool(value):
            if isinstance(value, str):
                return value.strip().lower() in ("1", "true", "yes", "on")
            return bool(value)

        visible = as_bool(data.get("tower", False)) and as_bool(data.get("conductors", False))
        valid = (
            visible
            and not as_bool(data.get("false_positive", False))
            and as_bool(data.get("depth_ready", False))
            and math.isfinite(quality)
            and math.isfinite(confidence)
            and quality >= self._threshold
            and confidence >= self._threshold
        )
        # Prefer the detector's camera-pose/view id.  The fallback keeps the
        # node compatible with older detectors that did not provide one.
        view_id = data.get("view_id")
        if view_id is not None:
            try:
                key = ("view", int(view_id))
            except (TypeError, ValueError):
                key = ("quality", as_bool(data.get("tower")), as_bool(data.get("conductors")), round(quality, 2), round(confidence, 2))
        else:
            key = ("quality", as_bool(data.get("tower")), as_bool(data.get("conductors")), round(quality, 2), round(confidence, 2))
        if valid and key != self._last_key:
            self._completed = min(self._cells, self._completed + 1)
        self._last_key = key
        out = String()
        out.data = (
            f"QUALITY_COVERAGE completed={self._completed}/{self._cells} "
            f"ratio={self._completed / self._cells:.3f} valid={str(valid).lower()} "
            f"quality={quality:.3f} confidence={confidence:.3f}"
        )
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = QualityCoverageSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
