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
#ifndef INCLUDE_COMPUTER_VISION_OPENCVSUBSCRIBER_HPP_
#define INCLUDE_COMPUTER_VISION_OPENCVSUBSCRIBER_HPP_

#include <image_transport/image_transport.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class OpenCVSubscriber : public rclcpp::Node
{
public:
  OpenCVSubscriber()
  : Node("opencv_subscriber")
  {
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_in", qos, std::bind(&OpenCVSubscriber::topic_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", qos);
  }

private:
  cv::Mat image_processing(const cv::Mat in_image) const;

  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    // Convert ROS Image to CV Image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image_raw = cv_ptr->image;

    // Image processing
    cv::Mat cv_image = image_processing(image_raw);

    // Convert OpenCV Image to ROS Image
    cv_bridge::CvImage img_bridge =
      cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, cv_image);

    // >> message to be sent
    sensor_msgs::msg::Image out_image;

    // from cv_bridge to sensor_msgs::Image
    img_bridge.toImageMsg(out_image);

    // Publish the data
    publisher_->publish(out_image);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

#endif  // INCLUDE_COMPUTER_VISION_OPENCVSUBSCRIBER_HPP_
