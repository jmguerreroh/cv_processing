/*
 # Copyright (c) 2023 José Miguel Guerrero Hernández
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 */

#include "computer_vision/OpenCVSubscriber.hpp"

/**
   TO-DO: Default - the output image is the same as the input
 */
cv::Mat OpenCVSubscriber::image_processing_rgb(const cv::Mat in_image) const
{
  // Create output image
  cv::Mat out_image;

  // Processing
  out_image = in_image;

  // Show image in a different window
  cv::imshow("out_image_rgb", out_image);
  cv::waitKey(3);

  // You must to return a 3-channels image to show it in ROS,
  // so do it with 1-channel images
  // cv::cvtColor(out_image, out_image, cv::COLOR_GRAY2BGR);
  return out_image;
}


/**
   TO-DO: Default - the output image is the same as the input
 */
cv::Mat OpenCVSubscriber::image_processing_depth(const cv::Mat in_image) const
{
  // Create output image
  cv::Mat out_image;

  // Processing
  out_image = in_image;

  // Show image in a different window
  cv::imshow("out_image_depth", out_image);
  cv::waitKey(3);

  // You must to return a 3-channels image to show it in ROS,
  // so do it with 1-channel images
  // cv::cvtColor(out_image, out_image, cv::COLOR_GRAY2BGR);
  return out_image;
}
