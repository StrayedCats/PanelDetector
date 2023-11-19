# PanelDetector
Panel detection node for CoRE-1 2024

## Node Structure

### Topic (Subscribe)

| Topic Name | Type | Description |
| --- | --- | --- |
| image_raw | sensor_msgs/msg/Image | Raw image from camera |

### Topic (Publish)

| Topic Name | Type | Description |
| --- | --- | --- |
| positions | geometry_msgs/msg/Detection2DArray | Detected panel poses |

### Class Diagram

```mermaid
---
title: Panel Detector Plugin hierarchy
---
classDiagram
    PanelDetectorNode <-- DetectorPluginA : load as dll
    PanelDetectorNode <-- DetectorPluginB : load as dll
    DetectorPluginA <|-- DetectorBase : include
    DetectorPluginB <|-- DetectorBase : include
    PanelDetectorNode <|-- DetectorBase : include
    PanelDetectorNode: params
    PanelDetectorNode: image_callback(Image)
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
