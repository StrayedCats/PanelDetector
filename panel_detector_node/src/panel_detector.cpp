#include <panel_detector_node/panel_detector_node.hpp>

namespace panel_detector_node
{

PanelDetectorNode::PanelDetectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("panel_detector_node", options),
  detection_loader_("panel_detector_base", "panel_detector_base::Detector")
{
  this->param_listener_ = std::make_shared<panel_detector_parameters::ParamListener>(
    this->get_node_parameters_interface());
  const auto params = this->param_listener_->get_params();

  try {
    std::cout << "params.load_target_plugin: " << params.load_target_plugin << std::endl;
    this->detector_ = this->detection_loader_.createSharedInstance(
      "panel_detector_plugins::PublishCenter");
    this->detector_->init(*this->param_listener_);
  } catch (pluginlib::PluginlibException & ex) {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  this->image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", 1, std::bind(&PanelDetectorNode::image_callback, this, std::placeholders::_1));
}

void PanelDetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  vision_msgs::msg::Detection2DArray bboxes =
    this->detector_->detect(cv_bridge::toCvShare(msg, "bgr8")->image);
  for (auto bbox : bboxes.detections) {
    std::cout << "bbox: " << bbox.bbox.center.position.x << ", " << bbox.bbox.center.position.y <<
      std::endl;
  }
}
} // namespace panel_detector_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(panel_detector_node::PanelDetectorNode)
