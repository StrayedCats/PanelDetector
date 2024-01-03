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

#include <detector2d_base/detector2d_base.hpp>
#include <detector2d_param/detector2d_param.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <memory>

namespace detector2d_plugins
{
typedef vision_msgs::msg::Detection2DArray Detection2DArray;
typedef std::vector<std::vector<cv::Point>> Contours;
typedef std::vector<cv::Point> CenterPoints;

class PanelSimpleDetector : public detector2d_base::Detector
{
public:
  void init(const detector2d_parameters::ParamListener &) override;
  Detection2DArray detect(const cv::Mat &) override;

private:
  void draw(const cv::Mat &, const CenterPoints &, const std::string &);
  CenterPoints get_center_points(const Contours &);
  CenterPoints merge_center_points(const CenterPoints &, const int &);

private:
  detector2d_parameters::Params params_;
};
}
