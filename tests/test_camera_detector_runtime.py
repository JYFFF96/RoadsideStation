from __future__ import print_function
import os
import sys
import tempfile
import types
import unittest
from unittest import mock

if "cv2" not in sys.modules:
    cv2_stub = types.ModuleType("cv2")
    cv2_stub.dnn = types.SimpleNamespace(readNetFromONNX=lambda path: object())
    sys.modules["cv2"] = cv2_stub

from roadside.camera_detector import YoloV5OnnxDetector


class _Input(object):
    name = "images"


class _Session(object):
    def __init__(self, path, providers=None):
        self.path = path
        self.providers = providers

    def get_inputs(self):
        return [_Input()]


class CameraDetectorRuntimeTest(unittest.TestCase):
    def setUp(self):
        handle, self.model = tempfile.mkstemp(suffix=".onnx")
        os.close(handle)

    def tearDown(self):
        os.unlink(self.model)

    @mock.patch("roadside.camera_detector.cv2.dnn.readNetFromONNX")
    def test_keeps_opencv_when_model_loads(self, load):
        load.return_value = object()
        detector = YoloV5OnnxDetector(self.model)
        self.assertEqual("opencv", detector.runtime)
        self.assertEqual("yolov5_onnx_opencv", detector.name)

    @mock.patch("roadside.camera_detector.cv2.dnn.readNetFromONNX")
    def test_falls_back_to_onnxruntime(self, load):
        load.side_effect = RuntimeError("Floor parse error")
        module = types.ModuleType("onnxruntime")
        module.InferenceSession = _Session
        with mock.patch.dict(sys.modules, {"onnxruntime": module}):
            detector = YoloV5OnnxDetector(self.model)
        self.assertEqual("onnxruntime", detector.runtime)
        self.assertEqual("images", detector.input_name)
        self.assertEqual(["CPUExecutionProvider"], detector.session.providers)

    @mock.patch("roadside.camera_detector.cv2.dnn.readNetFromONNX")
    def test_reports_install_command_when_both_runtimes_fail(self, load):
        load.side_effect = RuntimeError("Floor parse error")
        with mock.patch.dict(sys.modules, {"onnxruntime": None}):
            with self.assertRaises(RuntimeError) as caught:
                YoloV5OnnxDetector(self.model)
        self.assertIn("onnxruntime==1.14.1", str(caught.exception))


if __name__ == "__main__":
    unittest.main()
