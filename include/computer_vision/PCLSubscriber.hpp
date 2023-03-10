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
#ifndef INCLUDE_COMPUTER_VISION_PCLSUBSCRIBER_HPP_
#define INCLUDE_COMPUTER_VISION_PCLSUBSCRIBER_HPP_

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"

class PCLSubscriber : public rclcpp::Node
{
public:
  PCLSubscriber()
  : Node("pcl_subscriber")
  {
    subscription_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/pointcloud_in", rclcpp::SensorDataQoS().reliable(),
      std::bind(&PCLSubscriber::topic_callback_pointcloud, this, std::placeholders::_1));

    publisher_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "pointcloud",
      rclcpp::SensorDataQoS().reliable());
  }

private:
  pcl::PointCloud<pcl::PointXYZRGB> pcl_processing(
    const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud) const;

  void topic_callback_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    if (publisher_pointcloud_->get_subscription_count() > 0) { // Remove if you want to process it anyway
      // Convert to PCL data type
      pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
      pcl::fromROSMsg(*msg, point_cloud);

      pcl::PointCloud<pcl::PointXYZRGB> pcl_pointcloud = pcl_processing(point_cloud);

      // Convert to ROS data type
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(pcl_pointcloud, output);
      output.header = msg->header;

      // Publish the data
      publisher_pointcloud_->publish(output);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pointcloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pointcloud_;
};

#endif  // INCLUDE_COMPUTER_VISION_PCLSUBSCRIBER_HPP_
