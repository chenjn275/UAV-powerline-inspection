#!/usr/bin/env python3
"""Pure ROS sensor simulation for D435i-style images/depth and MID-360 cloud."""
import math
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class InspectionSensorSim(Node):
    def __init__(self):
        super().__init__('inspection_sensor_sim')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('tower_x', 0.0)
        self.declare_parameter('tower_y', 0.0)
        self.declare_parameter('tower_height_m', 10.0)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('depth_mm', 6000)
        self.frame = str(self.get_parameter('frame_id').value)
        self.cx = float(self.get_parameter('tower_x').value)
        self.cy = float(self.get_parameter('tower_y').value)
        self.height = float(self.get_parameter('tower_height_m').value)
        self.image_width = max(32, int(self.get_parameter('image_width').value))
        self.image_height = max(24, int(self.get_parameter('image_height').value))
        self.depth_mm = max(500, min(65535, int(self.get_parameter('depth_mm').value)))
        self._cloud_pub = self.create_publisher(PointCloud2, '/inspection/tower_points', 10)
        self._lidar_pub = self.create_publisher(PointCloud2, '/livox/lidar', 10)
        self._depth_pub = self.create_publisher(Image, '/inspection_d435i/depth/image_rect_raw', 10)
        self._color_pub = self.create_publisher(Image, '/inspection_d435i/color/image_raw', 10)
        self._seq = 0
        self.create_timer(1.0 / max(1.0, float(self.get_parameter('rate').value)), self.publish)

    def _tower_points(self):
        points = []
        for i in range(240):
            z = 0.15 + (i % 80) * ((self.height - 0.3) / 79.0)
            leg = i % 3
            angle = leg * 2.0 * math.pi / 3.0
            radius = 1.5 * (1.0 - 0.45 * z / self.height)
            points.append((self.cx + radius * math.cos(angle) + 0.02 * math.sin(i),
                           self.cy + radius * math.sin(angle) + 0.02 * math.cos(i), z))
        # A few conductor-like returns make the lidar stream less artificial.
        for i in range(30):
            points.append((self.cx + 3.0 + 0.02 * math.sin(i), self.cy - 0.8 + i * 0.05, 8.8))
        return points

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self.frame)
        cloud = point_cloud2.create_cloud_xyz32(header, self._tower_points())
        self._cloud_pub.publish(cloud)
        self._lidar_pub.publish(cloud)
        # Render a deterministic, lightweight camera view.  The tower is a
        # bright vertical silhouette in the middle of a textured background;
        # depth contains valid millimetre values only on the silhouette.
        # This keeps the simulation useful for readiness/occlusion checks and
        # avoids claiming that an all-zero image is a valid D435i frame.
        width, height = self.image_width, self.image_height
        cx, cy = width // 2, height // 2
        tower_half_width = max(3, width // 22)
        tower_top = max(2, height // 12)
        tower_bottom = height - max(2, height // 10)
        depth_bytes = bytearray(width * height * 2)
        rgb_bytes = bytearray(width * height * 3)
        for v in range(height):
            for u in range(width):
                # Mild spatial texture makes blur/contrast metrics meaningful.
                background = (24 + ((u + 3 * v) % 18), 62 + ((2 * u + v) % 20), 118 + ((u + v) % 24))
                is_tower = abs(u - cx) <= tower_half_width and tower_top <= v <= tower_bottom
                if is_tower:
                    color_rgb = (208, 208, 186)
                    depth_value = self.depth_mm + int(80.0 * math.sin(v * 0.08))
                else:
                    color_rgb = background
                    depth_value = 0
                rgb_index = (v * width + u) * 3
                rgb_bytes[rgb_index:rgb_index + 3] = bytes(color_rgb)
                depth_index = (v * width + u) * 2
                struct.pack_into('<H', depth_bytes, depth_index, max(0, min(65535, depth_value)))
        depth = Image(header=header, height=height, width=width, encoding='16UC1', is_bigendian=False,
                      step=width * 2)
        depth.data = bytes(depth_bytes)
        color = Image(header=header, height=height, width=width, encoding='rgb8', is_bigendian=False,
                      step=width * 3)
        color.data = bytes(rgb_bytes)
        self._depth_pub.publish(depth)
        self._color_pub.publish(color)
        self._seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = InspectionSensorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
