#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <panel_detector_base/panel_detector_base.hpp>
#include <panel_detector_param/panel_detector_param.hpp>
#include <panel_detector_plugins/publish_center_plugin.hpp>
#include <rclcpp/rclcpp.hpp>

class TestPanelDetectorClass : public ::testing::Test
{
protected:
  std::unique_ptr<panel_detector_plugins::PublishCenter> target_class_;
  std::shared_ptr<panel_detector_parameters::ParamListener> param_listener_;

  virtual void SetUp()
  {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("test_node");

    this->target_class_ =
      std::make_unique<panel_detector_plugins::PublishCenter>();

    this->param_listener_ =
      std::make_shared<panel_detector_parameters::ParamListener>(
      node->get_node_parameters_interface());
    this->target_class_->init(*this->param_listener_);
  }
};

TEST_F(TestPanelDetectorClass, test_default)
{
  auto result = this->target_class_->detect(cv::Mat(480, 640, CV_8UC3));
  EXPECT_EQ(result.detections[0].bbox.center.position.x, 320);
  EXPECT_EQ(result.detections[0].bbox.center.position.y, 240);
}
