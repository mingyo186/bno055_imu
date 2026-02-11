# Copyright 2025 The bno055_imu Authors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""Launch file for bno055_imu package."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description for the BNO055 IMU node."""
    pkg_dir = get_package_share_directory('bno055_imu')
    default_params = os.path.join(pkg_dir, 'config', 'bno055_params.yaml')

    return LaunchDescription([
        # -- Launch arguments (override on CLI) --------------------------------
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the parameter YAML file',
        ),

        # -- BNO055 IMU Node ---------------------------------------------------
        Node(
            package='bno055_imu',
            executable='bno055_node.py',
            name='bno055_imu_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('imu/data', 'imu/data'),
                ('imu/mag', 'imu/mag'),
                ('imu/temp', 'imu/temp'),
            ],
        ),
    ])
