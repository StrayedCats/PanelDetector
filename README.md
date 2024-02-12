# Detector2d
Detection 2d node for CoRE-1 2024

<img width="818" alt="スクリーンショット 2024-02-12 12 00 46" src="https://github.com/StrayedCats/detector2d/assets/67567093/ab8d1d5b-2bd8-4ef2-bf53-7f8a91d1ab38">

## test

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/StrayedCats/detector2d.git -b humble

cd ..
colcon build

source install/setup.bash
ros2 run detector2d_node detector2d_node_exec --ros-args -p load_target_plugin:=detector2d_plugins::PublishCenter
```

## Node Structure

### Topic (Subscribe)

| Topic Name | Type | Description |
| --- | --- | --- |
| image_raw | sensor_msgs/msg/Image | Raw image from camera |

### Topic (Publish)

| Topic Name | Type | Description |
| --- | --- | --- |
| positions | geometry_msgs/msg/Detection2DArray | Detected 2d poses |

### Class Diagram

```mermaid
---
title: Detector2d Plugin hierarchy
---
classDiagram
    Detector2dNode <-- DetectorPluginA : load as dll
    Detector2dNode <-- DetectorPluginB : load as dll
    DetectorPluginA <|-- DetectorBase : include
    DetectorPluginB <|-- DetectorBase : include
    Detector2dNode <|-- DetectorBase : include
    Detector2dNode: params
    Detector2dNode: image_callback(Image)
    class DetectorPluginA{
        filter_kernel_param_a
        init(params)
        processing(cv::Mat)
    }
    class DetectorPluginB{
        filter_kernel_param_b
        init(params)
        processing(cv::Mat)
    }
```
