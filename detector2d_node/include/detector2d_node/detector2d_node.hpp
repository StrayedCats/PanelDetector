#pragma once

#include <cv_bridge/cv_bridge.h>
#include <detector2d_base/detector2d_base.hpp>
#include <detector2d_param/detector2d_param.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace detector2d_node
{
class Detector2dNode : public rclcpp::Node
{
public:
  Detector2dNode(const rclcpp::NodeOptions &);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pose_pub_;
  pluginlib::ClassLoader<detector2d_base::Detector> detection_loader_;
  std::shared_ptr<detector2d_base::Detector> detector_;

  std::shared_ptr<detector2d_parameters::ParamListener> param_listener_;
};
}
