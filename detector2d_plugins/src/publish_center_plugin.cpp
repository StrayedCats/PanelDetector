#include "detector2d_plugins/publish_center_plugin.hpp"

namespace detector2d_plugins
{

void PublishCenter::init(const detector2d_parameters::ParamListener & param_listener)
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
PLUGINLIB_EXPORT_CLASS(detector2d_plugins::PublishCenter, detector2d_base::Detector)
