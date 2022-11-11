import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params = LaunchConfiguration(
    'params',
    default=os.path.join(
        get_package_share_directory('npu_camera'),
        'param',
        'cam_param.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'params',
            default_value=params,
            description='Full path of parameter file'),

        Node(
            package='npu_camera',
            executable='img_pub',
            name='img_pub',
            parameters=[params],
            output='screen'),

        Node(
            package='npu_camera',
            executable='img_sub',
            name='img_sub',
            parameters=[params],
            output='screen'),
        ])
