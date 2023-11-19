#include <panel_detector_base/panel_detector_base.hpp>
#include <panel_detector_param/panel_detector_param.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace panel_detector_plugins
{
class PublishCenter : public panel_detector_base::Detector
{
public:
  void init(const panel_detector_parameters::ParamListener & param_listener) override
  {
    (void)param_listener;
  }
  vision_msgs::msg::Detection2DArray detect(const cv::Mat & image) override
  {
    int col = image.cols;
    int row = image.rows;

    vision_msgs::msg::Detection2DArray pose;
    pose.detections.resize(1);
    pose.detections[0].bbox.center.position.x = col / 2;
    pose.detections[0].bbox.center.position.y = row / 2;
    return pose;
  }

protected:
  double side_length_;
};
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(panel_detector_plugins::PublishCenter, panel_detector_base::Detector)
