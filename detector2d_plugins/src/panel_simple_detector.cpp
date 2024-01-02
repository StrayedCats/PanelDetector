#include "detector2d_plugins/panel_simple_detector.hpp"

namespace detector2d_plugins
{

void PanelSimpleDetector::init(const detector2d_parameters::ParamListener & param_listener)
{
  this->params_ = param_listener.get_params();
}

Detection2DArray PanelSimpleDetector::detect(const cv::Mat & image)
{
  auto panel_color_ = this->params_.panel_simple_detector.panel_color;
  auto debug = this->params_.debug;

  cv::Mat3b hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat1b image_mask = cv::Mat1b::zeros(hsv.rows, hsv.cols);

  // red
  for (int i = 0; i < hsv.rows; i++) {
    for (int j = 0; j < hsv.cols; j++) {
      if ((hsv(i, j)[0] > 0 && hsv(i, j)[0] < 20) || (hsv(i, j)[0] > 170 && hsv(i, j)[0] < 180)) {
        image_mask(i, j) = hsv(i, j)[2] > 30 ? 255 : 0;
      } else {
        image_mask(i, j) = 0;
      }
    }
  }

  // TODO: blue

  cv::medianBlur(image_mask, image_mask, 7);

  // find contours
  Contours contours;
  cv::findContours(image_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  CenterPoints center_points = get_center_points(contours);
  CenterPoints merged_center_points = merge_center_points(center_points, image.cols);

  if (debug) {
    draw(image, merged_center_points, "merged_center_points");
  }

  Detection2DArray pose;
  for (auto center_point : merged_center_points) {
    vision_msgs::msg::Detection2D detection;
    detection.bbox.center.position.x = center_point.x;
    detection.bbox.center.position.y = center_point.y;
    detection.bbox.size_x = 10;
    detection.bbox.size_y = 10;
    detection.results.resize(1);
    pose.detections.push_back(detection);
  }
  return pose;
}

void PanelSimpleDetector::draw(const cv::Mat & image, const CenterPoints & center_points, const std::string & window_name)
{
  cv::Mat3b canvas = image.clone();
  for (auto center_point : center_points) {
    cv::circle(canvas, center_point, 2, cv::Scalar(0, 255, 0), 2);
  }
  cv::imshow(window_name, canvas);
  cv::waitKey(1);
}

CenterPoints PanelSimpleDetector::get_center_points(const Contours & contours)
{
  CenterPoints center_points;
  for (auto contour : contours) {
    cv::Rect rect = cv::boundingRect(contour);
    center_points.push_back(cv::Point(rect.x + rect.width / 2, rect.y + rect.height / 2));
  }
  return center_points;
}

CenterPoints PanelSimpleDetector::merge_center_points(const CenterPoints & center_points, const int & width)
{
  CenterPoints merged_center_points;
  uint32_t threshold = width * 0.05;
  for (auto center_point : center_points) {
    bool merged = false;
    for (size_t i = 0; i < merged_center_points.size(); i++) {
      if (abs(center_point.x - merged_center_points[i].x) < threshold) {
        merged_center_points[i].x = (center_point.x + merged_center_points[i].x) / 2;
        merged_center_points[i].y = (center_point.y + merged_center_points[i].y) / 2;
        merged = true;
        break;
      }
    }
    if (!merged) {
      merged_center_points.push_back(center_point);
    }
  }
  return merged_center_points;

}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(detector2d_plugins::PanelSimpleDetector, detector2d_base::Detector)
