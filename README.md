# Detector2d
Detection 2d node for CoRE-1 2024

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
