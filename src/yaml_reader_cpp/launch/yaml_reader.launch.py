import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包的 share 目录路径
    pkg_share = get_package_share_directory('yaml_reader_cpp')

    # 拼接 yaml 文件的完整路径
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='yaml_reader_cpp',
            executable='yaml_reader_node',
            name='yaml_reader_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': False,
                    'frames/odom': 'new_odom',
                    'frames/lidar': 'new_lidar',
                }
                ],
        )
    ])
