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
  this->params_ = this->param_listener_->get_params();

  try {
    this->detector_ = this->detection_loader_.createSharedInstance(
      this->params_.load_target_plugin);
    this->detector_->init(*this->param_listener_);
    std::cout << "params.load_target_plugin: " << this->params_.load_target_plugin << std::endl;
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s", ex.what());
  }

  this->pose_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
    "positions", 1);
  this->image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", rclcpp::SensorDataQoS(), std::bind(&Detector2dNode::image_callback, this, std::placeholders::_1));
}

void Detector2dNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    vision_msgs::msg::Detection2DArray bboxes =
      this->detector_->detect(cv_bridge::toCvShare(msg, "bgr8")->image);
    bboxes.header = msg->header;

    RCLCPP_INFO(this->get_logger(), "Detected %ld objects", bboxes.detections.size());

    if (this->params_.debug) {
      cv::imshow("detector", this->draw_bboxes(image, bboxes));
      auto key = cv::waitKey(1);
      if (key == 27) {
        rclcpp::shutdown();
      }
    }
    this->pose_pub_->publish(bboxes);
  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "exception: %s", e.what());
    return;
  }
}

cv::Mat3b Detector2dNode::draw_bboxes(
    const cv::Mat & frame,
    const vision_msgs::msg::Detection2DArray & boxes)
{
    cv::Mat3b frame_out;
    if (frame.channels() == 1)
    {
        cv::cvtColor(frame, frame_out, cv::COLOR_GRAY2BGR);
    }
    else {
        frame_out = frame;
    }

    for (auto detection : boxes.detections)
    {
        cv::Rect bbox;
        bbox.x = detection.bbox.center.position.x - detection.bbox.size_x / 2;
        bbox.y = detection.bbox.center.position.y - detection.bbox.size_y / 2;
        bbox.width = detection.bbox.size_x;
        bbox.height = detection.bbox.size_y;
        cv::rectangle(frame_out, bbox, cv::Scalar(0, 255, 0), 2);
    }
    return frame_out;
}

} // namespace detector2d_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(detector2d_node::Detector2dNode)
