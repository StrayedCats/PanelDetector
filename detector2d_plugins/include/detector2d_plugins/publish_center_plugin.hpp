#pragma once

#include <detector2d_base/detector2d_base.hpp>
#include <detector2d_param/detector2d_param.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace detector2d_plugins
{
typedef vision_msgs::msg::Detection2DArray Detection2DArray;
class PublishCenter : public detector2d_base::Detector
{
public:
  void init(const detector2d_parameters::ParamListener &) override;
  Detection2DArray detect(const cv::Mat &) override;
};
}
