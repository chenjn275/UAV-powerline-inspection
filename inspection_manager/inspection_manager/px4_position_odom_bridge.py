#!/usr/bin/env python3
"""Expose the PX4 bridge position as a stable ROS odometry contract.

The synthetic FAST-LIO input deliberately exercises the Livox/IMU pipeline,
but its inertial estimate can drift while PX4 tilts the aircraft in SITL.  The
planner still needs a bounded local pose to keep ROG-Map's virtual ground and
occupied-start checks meaningful.  The PX4 Offboard bridge already converts
the authoritative NED feedback into the configured ENU frame; this node makes
that pose available as ``/odom`` for SUPER while FAST-LIO's own estimate stays
available on its remapped diagnostic topic.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Px4PositionOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("px4_position_odom_bridge")
        self.declare_parameter("input_topic", "/px4_offboard_bridge/position_enu")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self._frame_id = str(self.get_parameter("frame_id").value).strip() or "odom"
        self._child_frame_id = (
            str(self.get_parameter("child_frame_id").value).strip() or "base_link"
        )
        self._pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("input_topic").value),
            self._on_position,
            10,
        )

    def _on_position(self, msg: PoseStamped) -> None:
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._child_frame_id
        odom.pose.pose = msg.pose
        self._pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4PositionOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
