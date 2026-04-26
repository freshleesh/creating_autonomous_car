from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # v4l2_camera node
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        output='screen',
        parameters=[
            {'video_device': '/dev/video0'},
            {'image_raw.compressed.format': 'jpeg'},
            {'image_raw.compressed.jpeg_quality': 30},
            {'qos_overrides./image_raw.publisher.reliability': 'best_effort'},
            {'qos_overrides./image_raw.publisher.durability': 'volatile'},
            {'qos_overrides./image_raw.publisher.history': 'keep_last'},
            {'qos_overrides./image_raw.publisher.depth': 1},
            {'qos_overrides./image_raw/compressed.publisher.reliability': 'best_effort'},
            {'qos_overrides./image_raw/compressed.publisher.durability': 'volatile'},
            {'qos_overrides./image_raw/compressed.publisher.history': 'keep_last'},
            {'qos_overrides./image_raw/compressed.publisher.depth': 1},
        ],
    )


    # livox_ros_driver2 rviz launch file
    livox_launch_dir = get_package_share_directory('livox_ros_driver2')
    rviz_launch_file = os.path.join(livox_launch_dir, 'launch_ROS2', 'rviz_MID360_launch.py')
    
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_file)
    )

    return LaunchDescription([
        v4l2_camera_node,
        livox_launch,
    ])
