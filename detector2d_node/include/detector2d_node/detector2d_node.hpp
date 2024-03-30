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

  detector2d_parameters::Detector2dParam params_;
};
}
