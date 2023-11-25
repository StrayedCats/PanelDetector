#pragma once

#include <panel_detector_base/panel_detector_base.hpp>
#include <panel_detector_param/panel_detector_param.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace panel_detector_plugins
{
typedef vision_msgs::msg::Detection2DArray Detection2DArray;
class PublishCenter : public panel_detector_base::Detector
{
public:
  void init(const panel_detector_parameters::ParamListener &) override;
  Detection2DArray detect(const cv::Mat &) override;
};
}
