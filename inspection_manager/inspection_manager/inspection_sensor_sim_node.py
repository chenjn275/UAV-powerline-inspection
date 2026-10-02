#!/usr/bin/env python3
"""Deterministic D435i/MID-360 sensor simulation.

The original demo published a world-frame :class:`PointCloud2` directly on
``/livox/lidar``.  That is useful for a geometry-only regression, but it is
not an input that FAST-LIO2 can consume: a real MID-360 publishes Livox
``CustomMsg`` packets in the lidar frame together with an IMU stream.  The
``fast_lio_input`` mode below produces that contract.  It uses PX4 local
position feedback when available (and a stationary, valid bootstrap pose
before PX4 starts publishing), transforms a fixed tower scene into the lidar
frame, and publishes realistic per-point time offsets.

The default mode remains the legacy deterministic PointCloud2 stream so old
unit tests and geometry-only demonstrations keep the same behaviour.
"""
import math
import struct
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

try:  # Available in the configured FAST-LIO2 workspace.
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
except ImportError:  # pragma: no cover - exercised only without the overlay
    CustomMsg = None
    CustomPoint = None

try:  # PX4 DDS is optional for the standalone sensor regression.
    from px4_msgs.msg import VehicleLocalPosition
except ImportError:  # pragma: no cover - exercised only without px4_msgs
    VehicleLocalPosition = None


class InspectionSensorSim(Node):
    def __init__(self):
        super().__init__('inspection_sensor_sim')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('fast_lio_input', False)
        self.declare_parameter('use_px4_odometry', True)
        # In FAST-LIO input mode these values describe the scene offset from
        # the PX4 local origin.  The vehicle pose itself is always expressed
        # in the PX4/ROS local frame; applying the offset to the vehicle pose
        # would cancel the initial target range and put the aircraft inside
        # the tower at startup.
        self.declare_parameter('sensor_origin_x', 6.0)
        self.declare_parameter('sensor_origin_y', 0.0)
        self.declare_parameter('sensor_origin_z', 0.0)
        self.declare_parameter('lidar_frame_id', 'livox_frame')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('px4_position_topic', '/fmu/out/vehicle_local_position')
        self.declare_parameter('tower_x', 0.0)
        self.declare_parameter('tower_y', 0.0)
        self.declare_parameter('tower_height_m', 10.0)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('imu_rate', 100.0)
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('depth_mm', 6000)
        self.frame = str(self.get_parameter('frame_id').value)
        self.fast_lio_input = bool(self.get_parameter('fast_lio_input').value)
        self.use_px4_odometry = bool(self.get_parameter('use_px4_odometry').value)
        self.sensor_origin = (
            float(self.get_parameter('sensor_origin_x').value),
            float(self.get_parameter('sensor_origin_y').value),
            float(self.get_parameter('sensor_origin_z').value),
        )
        self.lidar_frame = str(self.get_parameter('lidar_frame_id').value)
        self.imu_frame = str(self.get_parameter('imu_frame_id').value)
        self.cx = float(self.get_parameter('tower_x').value)
        self.cy = float(self.get_parameter('tower_y').value)
        self.height = float(self.get_parameter('tower_height_m').value)
        self.image_width = max(32, int(self.get_parameter('image_width').value))
        self.image_height = max(24, int(self.get_parameter('image_height').value))
        self.depth_mm = max(500, min(65535, int(self.get_parameter('depth_mm').value)))
        self._depth_frame, self._color_frame = self._build_camera_frames()
        self._cloud_pub = self.create_publisher(PointCloud2, '/inspection/tower_points', 10)
        self._lidar_pub = None
        self._livox_pub = None
        self._imu_pub = None
        self._fast_status_pub = None
        if self.fast_lio_input:
            if CustomMsg is None or CustomPoint is None:
                raise RuntimeError(
                    'fast_lio_input=true requires livox_ros_driver2/msg/CustomMsg; '
                    'source the FAST-LIO2 workspace before launching this node'
                )
            self._livox_pub = self.create_publisher(CustomMsg, '/livox/lidar', 10)
            self._imu_pub = self.create_publisher(Imu, '/livox/imu', 100)
            self._fast_status_pub = self.create_publisher(
                String,
                '/inspection/fast_lio/input_status',
                QoSProfile(
                    depth=1,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
        else:
            self._lidar_pub = self.create_publisher(PointCloud2, '/livox/lidar', 10)
        self._depth_pub = self.create_publisher(Image, '/inspection_d435i/depth/image_rect_raw', 10)
        self._color_pub = self.create_publisher(Image, '/inspection_d435i/color/image_raw', 10)
        self._seq = 0
        self._start_time = time.monotonic()
        self._px4_pose = None
        self._px4_pose_logged = False
        self._px4_accel_enu = (0.0, 0.0, 0.0)
        self._px4_yaw = 0.0
        self._last_pose_time = None
        self._last_yaw = 0.0
        self._yaw_rate = 0.0
        self._imu_callback_group = ReentrantCallbackGroup()
        if self.fast_lio_input and self.use_px4_odometry and VehicleLocalPosition is not None:
            px4_qos = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                VehicleLocalPosition,
                str(self.get_parameter('px4_position_topic').value),
                self._on_px4_position,
                px4_qos,
            )
        elif self.fast_lio_input and self.use_px4_odometry:
            self.get_logger().warning(
                'px4_msgs is unavailable; FAST-LIO input will use the stationary bootstrap pose'
            )
        self.create_timer(1.0 / max(1.0, float(self.get_parameter('rate').value)), self.publish)
        if self.fast_lio_input:
            self.create_timer(
                1.0 / max(10.0, float(self.get_parameter('imu_rate').value)),
                self.publish_imu,
                callback_group=self._imu_callback_group,
            )

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

    def _tower_points_world(self):
        """Return the fixed scene in the FAST-LIO map's world frame."""
        ox, oy, _ = self.sensor_origin
        return [
            (x + ox, y + oy, z)
            for x, y, z in self._tower_points()
        ]

    def _on_px4_position(self, msg):
        """Convert PX4 local NED position to the ROS ENU simulation frame."""
        values = (float(getattr(msg, 'x', math.nan)), float(getattr(msg, 'y', math.nan)),
                  float(getattr(msg, 'z', math.nan)))
        if not all(math.isfinite(value) for value in values):
            return
        if hasattr(msg, 'xy_valid') and not bool(msg.xy_valid):
            return
        if hasattr(msg, 'z_valid') and not bool(msg.z_valid):
            return
        # VehicleLocalPosition is NED: x=north, y=east, z=down.  Do not add
        # ``sensor_origin`` here: that parameter belongs to the fixed scene,
        # while this callback is the moving vehicle pose.
        east, north, up = values[1], values[0], -values[2]
        now = time.monotonic()
        heading = float(getattr(msg, 'heading', math.nan))
        if math.isfinite(heading):
            # PX4 heading is clockwise from north; ROS ENU yaw is CCW from east.
            self._px4_yaw = math.atan2(
                math.sin(math.pi / 2.0 - heading),
                math.cos(math.pi / 2.0 - heading),
            )
        if self._last_pose_time is not None and now > self._last_pose_time:
            self._yaw_rate = (self._px4_yaw - self._last_yaw) / (now - self._last_pose_time)
            self._yaw_rate = max(-3.0, min(3.0, self._yaw_rate))
        self._last_pose_time = now
        self._last_yaw = self._px4_yaw
        self._px4_pose = (east, north, up, self._px4_yaw)
        if not self._px4_pose_logged:
            self._px4_pose_logged = True
            self.get_logger().info(
                'FAST_LIO_SENSOR_POSE_CONNECTED source=PX4 vehicle_local_position '
                f'scene_offset=({self.sensor_origin[0]:.2f},{self.sensor_origin[1]:.2f},{self.sensor_origin[2]:.2f})'
            )
            if self._fast_status_pub is not None:
                self._fast_status_pub.publish(String(data='FAST_LIO_INPUT_READY source=PX4_ODOMETRY'))
        acceleration = (
            float(getattr(msg, 'ax', 0.0)),
            float(getattr(msg, 'ay', 0.0)),
            float(getattr(msg, 'az', 0.0)),
        )
        if all(math.isfinite(value) for value in acceleration):
            # PX4 acceleration is NED derivative of velocity. Convert to ENU.
            self._px4_accel_enu = (acceleration[1], acceleration[0], -acceleration[2])

    def _sensor_pose(self):
        """Return sensor position/yaw, with a valid bootstrap before PX4 DDS."""
        if self._px4_pose is not None:
            return self._px4_pose
        return 0.0, 0.0, 0.0, 0.0

    def _world_to_sensor(self, point, pose):
        sx, sy, sz, yaw = pose
        dx, dy = float(point[0]) - sx, float(point[1]) - sy
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * dx + s * dy, -s * dx + c * dy, float(point[2]) - sz)

    def _build_camera_frames(self):
        """Build the deterministic D435i buffers once instead of per scan."""
        width, height = self.image_width, self.image_height
        cx, cy = width // 2, height // 2
        tower_half_width = max(3, width // 22)
        tower_top = max(2, height // 12)
        tower_bottom = height - max(2, height // 10)
        depth_bytes = bytearray(width * height * 2)
        rgb_bytes = bytearray(width * height * 3)
        for v in range(height):
            for u in range(width):
                background = (24 + ((u + 3 * v) % 18), 62 + ((2 * u + v) % 20), 118 + ((u + v) % 24))
                is_tower = abs(u - cx) <= tower_half_width and tower_top <= v <= tower_bottom
                color_rgb = (208, 208, 186) if is_tower else background
                depth_value = self.depth_mm + int(80.0 * math.sin(v * 0.08)) if is_tower else 0
                rgb_index = (v * width + u) * 3
                rgb_bytes[rgb_index:rgb_index + 3] = bytes(color_rgb)
                depth_index = (v * width + u) * 2
                struct.pack_into('<H', depth_bytes, depth_index, max(0, min(65535, depth_value)))
        return bytes(depth_bytes), bytes(rgb_bytes)

    def _publish_fast_lio_scan(self, stamp):
        pose = self._sensor_pose()
        points_world = self._tower_points_world()
        msg = CustomMsg()
        msg.header = Header(stamp=stamp, frame_id=self.lidar_frame)
        msg.timebase = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        msg.lidar_id = 0
        msg.rsvd = [0, 0, 0]
        # Livox's offset_time is nanoseconds within one 100 ms scan.  The
        # real driver uses this field for motion undistortion.
        count = len(points_world)
        msg.points = []
        for index, world_point in enumerate(points_world):
            x, y, z = self._world_to_sensor(world_point, pose)
            point = CustomPoint()
            point.offset_time = int(100_000_000 * index / max(1, count - 1))
            point.x, point.y, point.z = float(x), float(y), float(z)
            point.reflectivity = 80 + (index % 120)
            point.tag = 0
            point.line = index % 6
            msg.points.append(point)
        msg.point_num = len(msg.points)
        assert self._livox_pub is not None
        self._livox_pub.publish(msg)

    def publish_imu(self):
        if not self.fast_lio_input or self._imu_pub is None:
            return
        stamp = self.get_clock().now().to_msg()
        imu = Imu()
        imu.header = Header(stamp=stamp, frame_id=self.imu_frame)
        # The simulated accelerometer reports specific force.  Add gravity
        # to PX4's world-frame velocity derivative and rotate into body FLU.
        ax, ay, az = self._px4_accel_enu
        _, _, _, yaw = self._sensor_pose()
        c, s = math.cos(yaw), math.sin(yaw)
        imu.linear_acceleration.x = c * ax + s * ay
        imu.linear_acceleration.y = -s * ax + c * ay
        imu.linear_acceleration.z = az + 9.81
        imu.angular_velocity.z = float(self._yaw_rate)
        self._imu_pub.publish(imu)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self.frame if not self.fast_lio_input else 'odom')
        cloud = point_cloud2.create_cloud_xyz32(header, self._tower_points())
        self._cloud_pub.publish(cloud)
        if self.fast_lio_input:
            self._publish_fast_lio_scan(stamp)
        else:
            assert self._lidar_pub is not None
            self._lidar_pub.publish(cloud)
        # Render a deterministic, lightweight camera view.  The tower is a
        # bright vertical silhouette in the middle of a textured background;
        # depth contains valid millimetre values only on the silhouette.
        # This keeps the simulation useful for readiness/occlusion checks and
        # avoids claiming that an all-zero image is a valid D435i frame.
        width, height = self.image_width, self.image_height
        depth = Image(header=header, height=height, width=width, encoding='16UC1', is_bigendian=False,
                      step=width * 2)
        depth.data = self._depth_frame
        color = Image(header=header, height=height, width=width, encoding='rgb8', is_bigendian=False,
                      step=width * 3)
        color.data = self._color_frame
        self._depth_pub.publish(depth)
        self._color_pub.publish(color)
        self._seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = InspectionSensorSim()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
