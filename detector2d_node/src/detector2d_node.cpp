// Copyright 2023 StrayedCats.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <detector2d_node/detector2d_node.hpp>

namespace detector2d_node
{

Detector2dNode::Detector2dNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("detector2d_node", options),
  detection_loader_("detector2d_base", "detector2d_base::Detector")
{
  this->param_listener_ = std::make_shared<detector2d_parameters::ParamListener>(
    this->get_node_parameters_interface());
  const auto params = this->param_listener_->get_params();

  try {
    this->detector_ = this->detection_loader_.createSharedInstance(
      params.load_target_plugin);
    this->detector_->init(*this->param_listener_);
    std::cout << "params.load_target_plugin: " << params.load_target_plugin << std::endl;
  } catch (pluginlib::PluginlibException & ex) {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  this->pose_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
    "positions", 1);
  this->image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", rclcpp::SensorDataQoS(), std::bind(&Detector2dNode::image_callback, this, std::placeholders::_1));
}

void Detector2dNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  auto start = std::chrono::system_clock::now();
  try {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "exception: %s", e.what());
    return;
  }

  vision_msgs::msg::Detection2DArray bboxes =
    this->detector_->detect(cv_bridge::toCvShare(msg, "bgr8")->image);
  // for (size_t i = 0; i < bboxes.detections.size(); i++) {
  //   std::cout << "bboxes [" << i << "]: " << bboxes.detections[i].bbox.center.position.x << ", " <<
  //     bboxes.detections[i].bbox.center.position.y <<
  //     std::endl;
  // }
  bboxes.header = msg->header;
  this->pose_pub_->publish(bboxes);
  auto end = std::chrono::system_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  RCLCPP_INFO(this->get_logger(), "elapsed time: %d ms", elapsed);
}
} // namespace detector2d_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(detector2d_node::Detector2dNode)
