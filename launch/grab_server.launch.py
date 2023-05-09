# Copyright 2021 Abrar Rahman Protyasha
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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_description_path = os.path.join(
                            get_package_share_directory("casia_bot_v1_description"),
                            'urdf/casia_bot_v1.urdf')

    print(robot_description_path)
    robot_description_content = open(robot_description_path).read()
    
    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription(
        [
            Node(
                package='grab_srvcli',
                executable='grab_server',
                output='screen',
                parameters=[robot_description],
            ),
        ]
    )
