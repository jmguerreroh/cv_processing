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
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_geometry/pinhole_camera_model.h"

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
    subscription_rgb_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_rgb_in", rclcpp::SensorDataQoS().reliable(),
      std::bind(&OpenCVSubscriber::topic_callback_rgb, this, std::placeholders::_1));

    publisher_rgb_ = this->create_publisher<sensor_msgs::msg::Image>(
      "image_rgb",
      rclcpp::SensorDataQoS().reliable());

    subscription_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_depth_in", rclcpp::SensorDataQoS().reliable(),
      std::bind(&OpenCVSubscriber::topic_callback_depth, this, std::placeholders::_1));

    publisher_depth_ = this->create_publisher<sensor_msgs::msg::Image>(
      "image_depth",
      rclcpp::SensorDataQoS().reliable());

    subscription_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", 1,
      std::bind(&OpenCVSubscriber::topic_callback_info, this, std::placeholders::_1));
  }

private:
  cv::Mat image_processing_rgb(const cv::Mat in_image) const;
  cv::Mat image_processing_depth(const cv::Mat in_image) const;

  void topic_callback_rgb(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    // Check if camera model has been received
    if (camera_model_ == nullptr) {
      RCLCPP_WARN(get_logger(), "Camera Model not yet available");
      return;
    }
    
    // Convert ROS Image to OpenCV Image | sensor_msgs::msg::Image -> cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat image_raw = cv_ptr->image;

    // Image processing
    cv::Mat cv_image = image_processing_rgb(image_raw);

    // Convert OpenCV Image to ROS Image
    cv_bridge::CvImage img_bridge =
      cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, cv_image);

    // >> message to be sent
    sensor_msgs::msg::Image out_image;

    // from cv_bridge to sensor_msgs::Image
    img_bridge.toImageMsg(out_image);

    // Publish the data
    publisher_rgb_->publish(out_image);
  }

  void topic_callback_depth(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    // Check if camera model has been received
    if (camera_model_ == nullptr) {
      RCLCPP_WARN(get_logger(), "Camera Model not yet available");
      return;
    }

    // Check if depth image has been received
    if (msg->encoding != "16UC1" && msg->encoding != "32FC1") {
      RCLCPP_ERROR(get_logger(), "The image type has not depth info");
      return;
    }

    // Convert ROS Image to OpenCV Image| sensor_msgs::msg::Image -> cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat image_raw = cv_ptr->image;

    // Image processing
    cv::Mat cv_image = image_processing_depth(image_raw);

    // Convert OpenCV Image to ROS Image
    cv_bridge::CvImage img_bridge =
      cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_32FC1, cv_image);

    // >> message to be sent
    sensor_msgs::msg::Image out_image;

    // from cv_bridge to sensor_msgs::Image
    img_bridge.toImageMsg(out_image);

    // Publish the data
    publisher_depth_->publish(out_image);
  }

  void topic_callback_info(sensor_msgs::msg::CameraInfo::UniquePtr msg)
  {
    RCLCPP_INFO(get_logger(), "Camera info received");

    camera_model_ = std::make_shared<image_geometry::PinholeCameraModel>();
    camera_model_->fromCameraInfo(*msg);
    std::cout << camera_model_->intrinsicMatrix() << std::endl;

    subscription_info_ = nullptr;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_rgb_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_rgb_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_info_;
  std::shared_ptr<image_geometry::PinholeCameraModel> camera_model_;
};

#endif  // INCLUDE_COMPUTER_VISION_OPENCVSUBSCRIBER_HPP_
