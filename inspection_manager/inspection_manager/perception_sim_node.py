#!/usr/bin/env python3
"""Synthetic D435i/MID-360 target observation for simulation-only integration."""

import math
import rclpy
from geometry_msgs.msg import PoseArray, Pose
from rclpy.node import Node


class SyntheticPerception(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_perception")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("tower_x", 0.0)
        self.declare_parameter("tower_y", 0.0)
        self.declare_parameter("tower_z", 5.0)
        self.declare_parameter("publish_rate", 2.0)
        self._frame = str(self.get_parameter("frame_id").value)
        self._x = float(self.get_parameter("tower_x").value)
        self._y = float(self.get_parameter("tower_y").value)
        self._z = float(self.get_parameter("tower_z").value)
        self._pub = self.create_publisher(PoseArray, "inspection/perception/targets", 10)
        self.create_timer(1.0 / max(0.1, float(self.get_parameter("publish_rate").value)), self._publish)

    def _publish(self) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self._x, self._y, self._z
        pose.orientation.w = 1.0
        msg.poses.append(pose)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyntheticPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
