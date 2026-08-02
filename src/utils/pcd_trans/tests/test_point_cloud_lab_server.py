import importlib.util
import math
import struct
import unittest
from pathlib import Path


SERVER_PATH = Path(__file__).resolve().parents[1] / "point_cloud_lab_server.py"


def load_server_module():
    spec = importlib.util.spec_from_file_location("point_cloud_lab_server", SERVER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PointCloudLabServerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = load_server_module()

    def test_decode_xyz_rejects_non_triplet_payload(self):
        with self.assertRaisesRegex(ValueError, "3 的倍数"):
            self.server.decode_xyz_f32(struct.pack("<ff", 1.0, 2.0))

    def test_decode_xyz_rejects_non_finite_coordinates(self):
        payload = struct.pack("<fff", 1.0, math.inf, 3.0)
        with self.assertRaisesRegex(ValueError, "非有限"):
            self.server.decode_xyz_f32(payload)

    def test_decode_xyz_returns_n_by_three_float32_array(self):
        payload = struct.pack("<ffffff", 1, 2, 3, 4, 5, 6)
        points = self.server.decode_xyz_f32(payload)
        self.assertEqual(points.shape, (2, 3))
        self.assertEqual(str(points.dtype), "float32")
        self.assertEqual(points[1].tolist(), [4.0, 5.0, 6.0])

    def test_filter_parameters_are_bounded(self):
        self.assertEqual(self.server.validate_statistical_params(20, 1.5), (20, 1.5))
        with self.assertRaisesRegex(ValueError, "neighbors"):
            self.server.validate_statistical_params(1, 1.5)
        with self.assertRaisesRegex(ValueError, "std_ratio"):
            self.server.validate_statistical_params(20, 0)

    def test_uploaded_filename_is_reduced_to_safe_suffix(self):
        self.assertEqual(self.server.safe_pcd_suffix("../../arena.PCD"), ".pcd")
        self.assertEqual(self.server.safe_pcd_suffix("cloud.bin"), ".pcd")


if __name__ == "__main__":
    unittest.main()
