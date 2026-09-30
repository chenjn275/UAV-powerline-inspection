#!/usr/bin/env python3
"""Deterministic synthetic tower point cloud for end-to-end ROS validation."""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class TowerPointCloudSim(Node):
    def __init__(self):
        super().__init__('tower_pointcloud_sim')
        self.declare_parameter('topic', '/inspection/tower_points')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('tower_x', 0.0)
        self.declare_parameter('tower_y', 0.0)
        self.declare_parameter('tower_height_m', 10.0)
        self.declare_parameter('rate', 2.0)
        self.pub = self.create_publisher(PointCloud2, str(self.get_parameter('topic').value), 10)
        self.frame = str(self.get_parameter('frame_id').value)
        self.tower_x = float(self.get_parameter('tower_x').value)
        self.tower_y = float(self.get_parameter('tower_y').value)
        self.tower_height = float(self.get_parameter('tower_height_m').value)
        if not math.isfinite(self.tower_height) or self.tower_height <= 3.0:
            raise ValueError('tower_height_m must be finite and greater than 3 m')
        self.t = 0
        self.create_timer(1.0 / max(0.1, float(self.get_parameter('rate').value)), self.publish)

    def publish(self):
        points = []
        # Three-leg lattice tower centered at the Gazebo tower origin.
        cx, cy = self.tower_x, self.tower_y
        for i in range(180):
            z = 0.15 + (i % 60) * ((self.tower_height - 0.3) / 59.0)
            leg = i % 3
            angle = leg * 2.0 * math.pi / 3.0
            radius = 1.5 * (1.0 - 0.45 * z / self.tower_height)
            x = cx + radius * math.cos(angle) + 0.03 * math.sin(i)
            y = cy + radius * math.sin(angle) + 0.03 * math.cos(i)
            points.append((x, y, z))
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame)
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, points))


def main(args=None):
    rclpy.init(args=args)
    node = TowerPointCloudSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
