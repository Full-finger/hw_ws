import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('turtlesim_control')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument('params_file', default_value=params_file),

        # 1. 启动 turtlesim 仿真器
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
        ),

        # 2. 启动控制节点（延迟1秒等 turtlesim 启动）
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='turtlesim_control',
                    executable='control_node',
                    name='control_node',
                    output='screen',
                    parameters=[
                        LaunchConfiguration('params_file'),
                    ],
                ),
            ]
        ),

        # 3. 启动遥控器节点（延迟2秒）
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='turtlesim_control',
                    executable='teleop_node',
                    name='teleop_node',
                    output='screen',
                ),
            ]
        ),
    ])
