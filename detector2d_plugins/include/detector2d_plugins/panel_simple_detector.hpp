#pragma once

#include <detector2d_base/detector2d_base.hpp>
#include <detector2d_param/detector2d_param.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <memory>

namespace detector2d_plugins
{
typedef vision_msgs::msg::Detection2DArray Detection2DArray;
typedef std::vector<std::vector<cv::Point>> Contours;
typedef std::vector<cv::Point> CenterPoints;

class PanelSimpleDetector : public detector2d_base::Detector
{
public:
  void init(const detector2d_parameters::ParamListener &) override;
  Detection2DArray detect(const cv::Mat &) override;

private:
  void draw(const cv::Mat &, const CenterPoints &, const std::string &);
  CenterPoints get_center_points(const Contours &);
  CenterPoints merge_center_points(const CenterPoints &, const int &);

private:
  detector2d_parameters::Params params_;
};
}
