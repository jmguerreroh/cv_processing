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

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='computer_vision',
            namespace='computer_vision',
            executable='cv_program',
            # Use topics from robot
            remappings=[
                ('/image_in', '/head_front_camera/rgb/image_raw'),
                ('/pointcloud_in', '/head_front_camera/depth_registered/points'),
            ],
            parameters=[{
                "opencv": False,
                "pcl": False,
            }]
        )
    ])