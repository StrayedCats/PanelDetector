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

  cv::Mat3b draw_bboxes(
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

protected:
  Detector() {}
};
}
