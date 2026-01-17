# ZED 3D Object Detector

This module provides 3D object detection using the ZED camera and Mask R-CNN, based on the [zed-pytorch](https://github.com/stereolabs/zed-pytorch) repository.

## Prerequisites

### 1. ZED SDK Installation
Install the ZED SDK and Python API following the official instructions:
- [ZED SDK Installation](https://www.stereolabs.com/docs/installation/)

### 2. PyTorch Installation
Install PyTorch with CUDA support. The CUDA version must match the one used for the ZED SDK.

**Using Conda (recommended):**
```bash
conda create --name zed_pytorch -y
conda activate zed_pytorch
conda install pytorch torchvision cudatoolkit=10.0 -c pytorch
```

**Using Pip:**
```bash
pip3 install torch torchvision
```

### 3. Mask R-CNN Installation
Install the maskrcnn-benchmark library:

```bash
git clone https://github.com/facebookresearch/maskrcnn-benchmark.git
cd maskrcnn-benchmark
python setup.py install
```

### 4. Apex Installation (Optional but recommended)
Install NVIDIA's Apex for mixed precision training:

```bash
git clone https://github.com/NVIDIA/apex
cd apex
python3 setup.py install
```

### 5. OpenCV
```bash
pip3 install opencv-python
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

### Running with ZED ROS Wrapper

If you're using the ZED ROS wrapper to publish camera topics:

1. Launch the ZED wrapper (in a separate terminal):
```bash
ros2 launch zed_wrapper zed2.launch.py
```

2. Run the 3D detector node:
```bash
ros2 run mapping_module zed_3d_detector \
    --config-file /path/to/configs/caffe2/e2e_mask_rcnn_R_50_C4_1x_caffe2.yaml \
    --min-image-size 256
```

### Running with ZED SDK Directly

The node can also work directly with the ZED SDK (without ROS wrapper):

```bash
ros2 run mapping_module zed_3d_detector \
    --config-file /path/to/configs/caffe2/e2e_mask_rcnn_R_50_C4_1x_caffe2.yaml \
    --min-image-size 256
```

### Running with SVO File

To replay a recorded SVO file:

```bash
ros2 run mapping_module zed_3d_detector \
    --config-file /path/to/config.yaml \
    --svo-filename /path/to/recording.svo \
    --min-image-size 256
```

### Command Line Arguments

- `--config-file`: Path to Mask R-CNN config file (required for Mask R-CNN)
- `--min-image-size`: Minimum image size for inference (default: 256)
- `--device`: Device to run inference on - 'cuda' or 'cpu' (default: 'cuda')
- `--show-mask-heatmaps`: Show mask heatmaps in visualization
- `--svo-filename`: Path to SVO file to replay instead of live camera

### Example Config Files

Download config files from the zed-pytorch repository:
- `configs/caffe2/e2e_mask_rcnn_R_50_C4_1x_caffe2.yaml`
- `configs/caffe2/e2e_mask_rcnn_R_101_FPN_1x_caffe2.yaml`
- `configs/caffe2/e2e_keypoint_rcnn_R_50_FPN_1x_caffe2.yaml`

## ROS2 Topics

### Subscribed Topics

- `/zed/zed_node/rgb/image_rect_color` (sensor_msgs/Image): RGB image from ZED camera
- `/zed/zed_node/depth/depth_registered` (sensor_msgs/Image): Depth image from ZED camera
- `/zed/zed_node/rgb/camera_info` (sensor_msgs/CameraInfo): Camera calibration parameters

### Published Topics

- `/zed_3d_detector/detections` (visualization_msgs/MarkerArray): 3D object detections as markers
- `/zed_3d_detector/annotated_image` (sensor_msgs/Image): Annotated image with bounding boxes

## Output

The node publishes:
- **3D positions** of detected objects in the camera frame
- **Bounding boxes** in 2D image space
- **Object labels** and confidence scores
- **Annotated images** with visualizations

## Troubleshooting

1. **Import errors**: Make sure all dependencies are installed in your Python environment
2. **CUDA errors**: Verify that your CUDA version matches between PyTorch and ZED SDK
3. **Camera not found**: Check that the ZED camera is connected and ZED SDK is properly installed
4. **No detections**: Ensure the config file path is correct and the model weights are downloaded

## References

- [zed-pytorch GitHub](https://github.com/stereolabs/zed-pytorch)
- [ZED SDK Documentation](https://www.stereolabs.com/docs/)
- [Mask R-CNN Benchmark](https://github.com/facebookresearch/maskrcnn-benchmark)
