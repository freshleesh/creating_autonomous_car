import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    livox_launch_file = os.path.join(
        get_package_share_directory("livox_ros_driver2"),
        "launch_ROS2",
        "rviz_MID360_launch.py",
    )
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_file)
    )

    # v4l2_camera 디바이스 경로 설정 가능
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Path to video device (e.g., /dev/video0, /dev/video1)'
    )

    v4l2_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        output="screen",
        parameters=[
            {'video_device': LaunchConfiguration('video_device')}
        ]
    )

    return LaunchDescription(
        [
            video_device_arg,
            v4l2_camera_node,
            livox_launch,
        ]
    )
