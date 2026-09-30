#!/usr/bin/env python3
"""Estimate a transmission tower pose from a cropped PointCloud2 stream.

The node intentionally performs geometry only. Semantic cropping can be
provided by a detector upstream; invalid and sparse clouds are reported as a
diagnostic status instead of producing stale poses.
"""

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from inspection_core import estimate_tower_observation


class TowerPointCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("tower_pointcloud_localizer")
        self.declare_parameter("input_topic", "/inspection/tower_points")
        self.declare_parameter("center_topic", "/inspection/tower/center")
        self.declare_parameter("extent_topic", "/inspection/tower/extent")
        self.declare_parameter("status_topic", "/inspection/tower/status")
        self.declare_parameter("marker_topic", "/inspection/tower/marker")
        self.declare_parameter("min_points", 30)
        self.declare_parameter("min_height_m", 2.0)
        self._min_points = int(self.get_parameter("min_points").value)
        self._min_height = float(self.get_parameter("min_height_m").value)
        self._center_pub = self.create_publisher(PoseStamped, str(self.get_parameter("center_topic").value), 10)
        self._extent_pub = self.create_publisher(Vector3Stamped, str(self.get_parameter("extent_topic").value), 10)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self._marker_pub = self.create_publisher(Marker, str(self.get_parameter("marker_topic").value), 10)
        self.create_subscription(PointCloud2, str(self.get_parameter("input_topic").value), self._cloud_callback, 10)

    @staticmethod
    def points_from_cloud(msg: PointCloud2):
        """Return finite xyz tuples; raises ValueError when fields are absent."""
        names = {field.name for field in msg.fields}
        if not {"x", "y", "z"}.issubset(names):
            raise ValueError("PointCloud2 must contain x, y and z fields")
        rows = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=False
        )
        # sensor_msgs_py on Humble returns a structured NumPy array. Its rows
        # are scalar records, so positional slices such as row[:3] fail.
        field_names = getattr(getattr(rows, "dtype", None), "names", None)
        if field_names:
            return [
                (float(row["x"]), float(row["y"]), float(row["z"]))
                for row in rows.reshape(-1)
            ]
        # Keep compatibility with older sensor_msgs_py releases that yielded
        # ordinary tuples instead of a structured array.
        return [tuple(float(value) for value in row[:3]) for row in rows]

    def _cloud_callback(self, msg: PointCloud2) -> None:
        try:
            observation = estimate_tower_observation(
                self.points_from_cloud(msg), min_points=self._min_points, min_height_m=self._min_height
            )
        except (ValueError, TypeError, IndexError, AssertionError) as exc:
            self._status_pub.publish(String(data=f"REJECTED: {exc}"))
            self.get_logger().warning(f"Rejected tower point cloud: {exc}")
            return
        center = PoseStamped()
        center.header = msg.header
        center.pose.position.x = observation.center_x
        center.pose.position.y = observation.center_y
        center.pose.position.z = (observation.base_z + observation.top_z) / 2.0
        center.pose.orientation.w = 1.0
        self._center_pub.publish(center)
        extent = Vector3Stamped()
        extent.header = msg.header
        extent.vector.x = observation.height_m
        extent.vector.y = observation.uncertainty_m
        extent.vector.z = float(observation.point_count)
        self._extent_pub.publish(extent)
        self._status_pub.publish(String(data="VALID"))
        marker = Marker()
        marker.header = msg.header
        marker.ns, marker.id, marker.type, marker.action = "tower_localization", 0, Marker.CYLINDER, Marker.ADD
        marker.pose = center.pose
        marker.scale.x = marker.scale.y = max(0.2, observation.uncertainty_m * 2.0)
        marker.scale.z = observation.height_m
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.8, 0.2, 0.65
        marker.lifetime.sec = 1
        self._marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TowerPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
