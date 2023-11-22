#include "panel_detector_plugins/publish_center.hpp"

namespace panel_detector_plugins
{

void PublishCenter::init(const panel_detector_parameters::ParamListener & param_listener)
{
  (void)param_listener;
}
Detection2DArray PublishCenter::detect(const cv::Mat & image)
{
  int col = image.cols;
  int row = image.rows;

  Detection2DArray pose;
  pose.detections.resize(1);
  pose.detections[0].bbox.center.position.x = col / 2;
  pose.detections[0].bbox.center.position.y = row / 2;
  return pose;
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(panel_detector_plugins::PublishCenter, panel_detector_base::Detector)
