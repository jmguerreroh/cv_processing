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

#include <memory>

#include "computer_vision/OpenCVSubscriber.hpp"
#include "computer_vision/PCLSubscriber.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  auto cv_node = std::make_shared<OpenCVSubscriber>();
  auto pcl_node = std::make_shared<PCLSubscriber>();

  exec.add_node(cv_node);
  exec.add_node(pcl_node);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
