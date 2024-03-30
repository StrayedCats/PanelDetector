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

#include "detector2d_plugins/publish_center_plugin.hpp"

namespace detector2d_plugins
{

void PublishCenter::init(const detector2d_parameters::ParamListener & param_listener)
{
  (void)param_listener;
}
Detection2DArray PublishCenter::detect(const cv::Mat & image)
{
  int col = image.cols;
  int row = image.rows;

  Detection2DArray pose;
  pose.detections.resize(1);
  pose.detections[0].bbox.center.position.x = col / 2;
  pose.detections[0].bbox.center.position.y = row / 2;
  pose.detections[0].bbox.size_x = 10;
  pose.detections[0].bbox.size_y = 10;
  return pose;
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(detector2d_plugins::PublishCenter, detector2d_base::Detector)
