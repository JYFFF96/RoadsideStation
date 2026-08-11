# Camera detection model

RoadsideStation V0.4 expects an OpenCV-DNN compatible YOLOv5 ONNX model at:

    models/yolov5n.onnx

The runtime does not require PyTorch; it loads the ONNX file through OpenCV.

One way to create the file is to export YOLOv5n to ONNX in a separate YOLOv5 environment, then copy the generated `yolov5n.onnx` into this directory.

The detector currently keeps road-related COCO classes only: person, bicycle, car, motorcycle, bus and truck.
