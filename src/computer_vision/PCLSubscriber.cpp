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

#include "computer_vision/PCLSubscriber.hpp"

/**
  TO-DO: Default - the output pointcloud is the same as the input
*/
pcl::PointCloud<pcl::PointXYZRGB> PCLSubscriber::pcl_processing(
    const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud) const {
    pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud;
    out_pointcloud = in_pointcloud;
    return out_pointcloud;
}

