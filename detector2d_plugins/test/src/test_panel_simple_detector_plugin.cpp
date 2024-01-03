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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <detector2d_base/detector2d_base.hpp>
#include <detector2d_param/detector2d_param.hpp>
#include <detector2d_plugins/panel_simple_detector.hpp>
#include <rclcpp/rclcpp.hpp>

class TestDetector2dClass : public ::testing::Test
{
protected:
  std::unique_ptr<detector2d_plugins::PanelSimpleDetector> target_class_;
  std::shared_ptr<detector2d_parameters::ParamListener> param_listener_;

  std::string image_path;

  virtual void SetUp()
  {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("test_node");

    this->image_path = ament_index_cpp::get_package_share_directory("detector2d_plugins") +
      "/test_data/dark_00.png";

    this->target_class_ =
      std::make_unique<detector2d_plugins::PanelSimpleDetector>();

    this->param_listener_ =
      std::make_shared<detector2d_parameters::ParamListener>(
      node->get_node_parameters_interface());
    this->target_class_->init(*this->param_listener_);
  }
};

TEST_F(TestDetector2dClass, test_default)
{
  cv::Mat3b image = cv::imread(this->image_path, cv::IMREAD_COLOR);
  auto result = this->target_class_->detect(image);

  // bboxes [0]: 468, 328
  // bboxes [1]: 632, 325
  // bboxes [2]: 882, 345 <-- remove
  // bboxes [3]: 797, 298
  // bboxes [4]: 555, 228

  ASSERT_EQ(result.detections[0].bbox.center.position.x, 468);
  ASSERT_EQ(result.detections[0].bbox.center.position.y, 328);

  ASSERT_EQ(result.detections[1].bbox.center.position.x, 632);
  ASSERT_EQ(result.detections[1].bbox.center.position.y, 325);

  ASSERT_EQ(result.detections[3].bbox.center.position.x, 797);
  ASSERT_EQ(result.detections[3].bbox.center.position.y, 298);

  ASSERT_EQ(result.detections[4].bbox.center.position.x, 555);
  ASSERT_EQ(result.detections[4].bbox.center.position.y, 228);
}
