# ZED 2D Object Detector with Custom Detector

This module provides 2D object detection using a custom bounding box detector (e.g., YOLO) integrated with the ZED SDK's custom object detection API. The 2D detections are ingested into the ZED SDK, which computes 3D positions, bounding boxes, and tracks objects over time.

Based on the [ZED SDK Custom Object Detection documentation](https://www.stereolabs.com/docs/object-detection/custom-od).

## How It Works

1. **2D Detection**: A custom 2D bounding box detector (e.g., YOLO) detects objects in the image
2. **Ingestion**: The 2D bounding boxes are ingested into the ZED SDK using `ingestCustomBoxObjects`
3. **3D Computation**: The ZED SDK computes 3D positions, 3D bounding boxes, and object masks using depth information
4. **Tracking**: If positional tracking is enabled, objects are tracked across frames with consistent IDs

## Prerequisites

### 1. ZED SDK Installation
Install the ZED SDK and Python API following the official instructions:
- [ZED SDK Installation](https://www.stereolabs.com/docs/installation/)

The ZED SDK is required for this module.

### 2. YOLO Installation (Optional but Recommended)
The default detector uses Ultralytics YOLO. Install it with:

```bash
pip install ultralytics
```

Alternatively, you can use any other 2D bounding box detector by modifying the `SimpleYOLODetector` class in `zed_2d_detector_node.py`.

### 3. PyTorch Installation
If using YOLO, install PyTorch:

```bash
pip install torch torchvision
```

## Building the ROS2 Package

1. Navigate to your workspace:
```bash
cd /home/khw/Dev/rover_ROS_ws
```

2. Build the package:
```bash
colcon build --packages-select mapping_module
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Running with Default YOLOv8 Model

The simplest way to run the detector is with the default YOLOv8n model:

```bash
ros2 run mapping_module zed_2d_detector
```

### Running with Custom YOLO Model

To use a custom YOLO model:

```bash
ros2 run mapping_module zed_2d_detector \
    --model-path /path/to/your/model.pt \
    --confidence-threshold 0.5
```

### Running with SVO File

To replay a recorded SVO file:

```bash
ros2 run mapping_module zed_2d_detector \
    --svo-filename /path/to/recording.svo \
    --model-path /path/to/model.pt
```

### Command Line Arguments

- `--model-path`: Path to YOLO model file (e.g., yolov8n.pt). If not provided, uses default YOLOv8n model.
- `--confidence-threshold`: Confidence threshold for detections (default: 0.5)
- `--device`: Device to run inference on - 'cuda' or 'cpu' (default: 'cuda')
- `--disable-tracking`: Disable object tracking (tracking is enabled by default)
- `--svo-filename`: Path to SVO file to replay instead of live camera
- `--resolution`: Camera resolution - 'HD2K', 'HD1080', 'HD720', or 'VGA' (default: 'HD720')

### Example: Running with Custom Settings

```bash
ros2 run mapping_module zed_2d_detector \
    --model-path yolov8m.pt \
    --confidence-threshold 0.6 \
    --device cuda \
    --resolution HD1080
```

## ROS2 Topics

### Published Topics

- `/zed_2d_detector/detections` (visualization_msgs/MarkerArray): 3D object detections as markers
  - Includes 3D positions, bounding boxes, and labels
  - Objects are tracked with consistent IDs across frames
  
- `/zed_2d_detector/annotated_image` (sensor_msgs/Image): Annotated image with bounding boxes
  - Shows 2D bounding boxes from the custom detector (green)
  - Shows 3D tracked objects with IDs and positions (red)

## Output

The node publishes:
- **3D positions** of detected objects in the camera frame
- **3D bounding boxes** computed from 2D detections and depth
- **Object IDs** for tracked objects (if tracking enabled)
- **Object labels** and confidence scores from the custom detector
- **Annotated images** with visualizations

## Customizing the Detector

To use a different 2D detector (not YOLO), modify the `SimpleYOLODetector` class in `zed_2d_detector_node.py`. The detector must implement a `detect()` method that returns a list of detections, each with:

- `bbox`: [x1, y1, x2, y2] bounding box coordinates
- `conf`: confidence score (0.0 to 1.0)
- `class_id`: integer class ID

Example:

```python
def detect(self, image):
    # Your detection code here
    detections = []
    for detection in your_detector_results:
        detections.append({
            'bbox': [x1, y1, x2, y2],
            'conf': confidence_score,
            'class_id': class_id
        })
    return detections
```

## Troubleshooting

1. **ZED SDK not found**: Make sure the ZED SDK is installed and the Python API is available
2. **YOLO import errors**: Install ultralytics: `pip install ultralytics`
3. **CUDA errors**: Verify that your CUDA version matches between PyTorch and ZED SDK
4. **Camera not found**: Check that the ZED camera is connected and ZED SDK is properly installed
5. **No detections**: Ensure the model path is correct and the model weights are downloaded

## Differences from 3D Detector

The 2D detector differs from the 3D detector in several ways:

- **Uses ZED SDK Custom Detection API**: The 2D detector uses `ingestCustomBoxObjects` to feed detections into the ZED SDK, which then computes 3D information
- **Flexible Detector**: Can use any 2D bounding box detector (YOLO, SSD, etc.)
- **SDK-based Tracking**: Object tracking is handled by the ZED SDK, providing consistent IDs across frames
- **Automatic 3D Computation**: The ZED SDK automatically computes 3D positions and bounding boxes from 2D detections and depth

## References

- [ZED SDK Custom Object Detection Documentation](https://www.stereolabs.com/docs/object-detection/custom-od)
- [ZED SDK Documentation](https://www.stereolabs.com/docs/)
- [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)
