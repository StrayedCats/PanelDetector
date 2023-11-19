#pragma once

#include <cv_bridge/cv_bridge.h>
#include <panel_detector_base/panel_detector_base.hpp>
#include <panel_detector_param/panel_detector_param.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace panel_detector_node
{
class PanelDetectorNode : public rclcpp::Node
{
public:
  PanelDetectorNode(const rclcpp::NodeOptions &);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pose_pub_;
  pluginlib::ClassLoader<panel_detector_base::Detector> detection_loader_;
  std::shared_ptr<panel_detector_base::Detector> detector_;

  std::shared_ptr<panel_detector_parameters::ParamListener> param_listener_;
};
}
