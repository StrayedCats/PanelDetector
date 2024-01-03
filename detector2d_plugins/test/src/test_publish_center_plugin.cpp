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

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <detector2d_base/detector2d_base.hpp>
#include <detector2d_param/detector2d_param.hpp>
#include <detector2d_plugins/publish_center_plugin.hpp>
#include <rclcpp/rclcpp.hpp>

class TestDetector2dClass : public ::testing::Test
{
protected:
  std::unique_ptr<detector2d_plugins::PublishCenter> target_class_;
  std::shared_ptr<detector2d_parameters::ParamListener> param_listener_;

  virtual void SetUp()
  {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("test_node");

    this->target_class_ =
      std::make_unique<detector2d_plugins::PublishCenter>();

    this->param_listener_ =
      std::make_shared<detector2d_parameters::ParamListener>(
      node->get_node_parameters_interface());
    this->target_class_->init(*this->param_listener_);
  }
};

TEST_F(TestDetector2dClass, test_default)
{
  auto result = this->target_class_->detect(cv::Mat(480, 640, CV_8UC3));
  EXPECT_EQ(result.detections[0].bbox.center.position.x, 320);
  EXPECT_EQ(result.detections[0].bbox.center.position.y, 240);
}
