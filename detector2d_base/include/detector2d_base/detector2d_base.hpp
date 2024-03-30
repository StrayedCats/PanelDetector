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

#include <detector2d_param/detector2d_param.hpp>
#include <vector>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <opencv2/opencv.hpp>

namespace detector2d_base
{
class Detector
{
public:
  virtual void init(const detector2d_parameters::ParamListener & param_listener) = 0;
  virtual vision_msgs::msg::Detection2DArray detect(const cv::Mat & image) = 0;
  virtual ~Detector() {}

protected:
  Detector() {}
};
}
