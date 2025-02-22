# Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    share_dir = get_package_share_directory('vox_nav_bringup')

    map_params = LaunchConfiguration('map_params')
    planner_params = LaunchConfiguration('planner_params')

    declare_map_params = DeclareLaunchArgument(
        'map_params',
        default_value=os.path.join(share_dir, 'params', 'map_uneven_world_params.yaml'),
        description='Path to the vox_nav map parameters file.')
    declare_planner_params = DeclareLaunchArgument(
        'planner_params',
        default_value=os.path.join(share_dir, 'params', 'planner_se2_params.yaml'),
        description='Path to the vox_nav planner parameters file.')

    map_server_node = Node(
        package='vox_nav_map_server',
        executable='map_manager_no_gps',
        name='vox_nav_map_server_rclcpp_node',
        namespace='',
        output='screen',
        parameters=[map_params],
    )
    planner_server_node = Node(
        package='vox_nav_planning',
        executable='planner_server',
        name='vox_nav_planner_server_rclcpp_node',
        namespace='',
        output='screen',
        parameters=[planner_params],
    )

    return LaunchDescription([
        declare_map_params,
        declare_planner_params,
        map_server_node,
        planner_server_node,
    ])
