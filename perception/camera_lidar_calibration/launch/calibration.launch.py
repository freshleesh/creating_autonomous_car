import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('camera_lidar_calibration')
    source_config_dir = os.path.join(package_dir, 'config')
    general_file = os.path.join(source_config_dir, 'general.yaml')
    params_file = os.path.join(source_config_dir, 'camera_extrinsic_calibration.yaml')
    intrinsic_file = os.path.join(source_config_dir, 'camera_intrinsic_calibration.yaml')

    # src 경로도 함께 저장하기 위해 추가
    src_config_dir = os.path.join(
        os.path.dirname(package_dir), '..', '..', 'src', 'perception',
        'camera_lidar_calibration', 'config'
    )
    src_general_file = os.path.join(src_config_dir, 'general.yaml')
    src_params_file = os.path.join(src_config_dir, 'camera_extrinsic_calibration.yaml')

    node = Node(
        package='camera_lidar_calibration',
        executable='camera_lidar_calibration',
        name='extrinsic_calibration_by_hand',
        output='screen',
        parameters=[
            general_file,
            params_file,
            {
                'camera_intrinsic_yaml': intrinsic_file,
                'save_general_yaml': general_file,
                'save_params_yaml': params_file,
                'save_general_yaml_src': src_general_file,
                'save_params_yaml_src': src_params_file,
                'save_on_enter': True,
            },
        ],
    )

    return LaunchDescription([node])
