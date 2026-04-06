from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_launch_description():
    package_path = get_package_share_directory('lm_calibr')
    config_file = os.path.join(package_path, 'config', 'transform_cloud.yaml')

    with open(config_file, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f) or {}

    return LaunchDescription([
        Node(
            package='lm_calibr',
            executable='transform_cloud_node',
            name='transform_cloud_node',
            output='screen',
            parameters=[params],
        )
    ])
