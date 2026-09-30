#!/usr/bin/env python3
"""Publish a moving spherical obstacle as a PointCloud2 stream."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def moving_obstacle_center(time_s: float, *, center_x: float = 4.0,
                           center_y: float = 0.0, center_z: float = 4.0,
                           amplitude: float = 3.0,
                           angular_speed: float = 0.5) -> tuple[float, float, float]:
    """Return the deterministic dynamic-obstacle center in ``map``.

    The default amplitude deliberately traverses both sides of the nominal
    orbit.  This makes the launch demonstrate both branches of replanning:
    a close obstacle forces a larger candidate radius, then a clear crossing
    allows the planner to return to the preferred small radius.
    """
    values = (time_s, center_x, center_y, center_z, amplitude, angular_speed)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("dynamic obstacle parameters must be finite")
    return (
        float(center_x) + float(amplitude) * math.sin(float(angular_speed) * float(time_s)),
        float(center_y),
        float(center_z),
    )


class ObstaclePointCloudSim(Node):
    def __init__(self):
        super().__init__('obstacle_pointcloud_sim')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('center_x', 4.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('center_z', 4.0)
        self.declare_parameter('amplitude', 3.0)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('radius', 0.6)
        self.declare_parameter('point_count', 96)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.center = tuple(float(self.get_parameter(name).value) for name in
                            ('center_x', 'center_y', 'center_z'))
        self.amplitude = float(self.get_parameter('amplitude').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.radius = max(0.0, float(self.get_parameter('radius').value))
        self.point_count = max(12, int(self.get_parameter('point_count').value))
        self.pub = self.create_publisher(PointCloud2, '/inspection/obstacles/points', 10)
        self.t = 0.0
        self.create_timer(0.1, self.publish)

    def publish(self):
        # Obstacle crosses the nominal orbit near the middle of the line.
        cx, cy, cz = moving_obstacle_center(
            self.t, center_x=self.center[0], center_y=self.center[1],
            center_z=self.center[2], amplitude=self.amplitude,
            angular_speed=self.angular_speed,
        )
        points = []
        latitude_count = max(2, self.point_count // 8)
        for i in range(self.point_count):
            a = 2.0 * math.pi * i / self.point_count
            b = math.pi * (i % latitude_count) / (latitude_count - 1)
            points.append((cx + self.radius * math.sin(b) * math.cos(a),
                           cy + self.radius * math.sin(b) * math.sin(a),
                           cz + self.radius * math.cos(b)))
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame_id)
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        self.t += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePointCloudSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
