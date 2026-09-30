import unittest

from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from inspection_manager.tower_pointcloud_node import TowerPointCloudNode


class PointCloudParsingTest(unittest.TestCase):
    def test_parses_humble_structured_point_records(self):
        msg = point_cloud2.create_cloud_xyz32(
            Header(frame_id="map"),
            [(1.0, 2.0, 3.0), (-4.0, 5.5, 6.25)],
        )
        self.assertEqual(
            TowerPointCloudNode.points_from_cloud(msg),
            [(1.0, 2.0, 3.0), (-4.0, 5.5, 6.25)],
        )

    def test_rejects_cloud_without_xyz_fields(self):
        msg = point_cloud2.create_cloud_xyz32(Header(frame_id="map"), [(0.0, 0.0, 0.0)])
        msg.fields = msg.fields[:2]
        with self.assertRaisesRegex(ValueError, "x, y and z"):
            TowerPointCloudNode.points_from_cloud(msg)


if __name__ == "__main__":
    unittest.main()
