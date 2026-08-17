from __future__ import print_function
import unittest
from roadside.camera_runtime_config import apply_camera_runtime_overrides

class CameraRuntimeConfigTest(unittest.TestCase):
    def test_overrides_are_copy_on_write(self):
        original={"camera_fusion":{"source":"carla_truth"},"camera_detection":{"model":"old.onnx"}}
        result=apply_camera_runtime_overrides(original,"detector","new.onnx")
        self.assertEqual("detector",result["camera_fusion"]["source"])
        self.assertEqual("new.onnx",result["camera_detection"]["model"])
        self.assertEqual("carla_truth",original["camera_fusion"]["source"])
        self.assertEqual("old.onnx",original["camera_detection"]["model"])

    def test_none_preserves_values(self):
        original={"camera_fusion":{"source":"carla_truth"},"camera_detection":{"model":"model.onnx"}}
        self.assertEqual(original,apply_camera_runtime_overrides(original))

if __name__=="__main__":unittest.main()
