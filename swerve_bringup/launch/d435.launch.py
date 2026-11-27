from pathlib import Path

import yaml
from launch_ros.actions import Node
from launch import LaunchDescription


def _base_camera_serial():
    """'_<SN>' from config/camera_serials.yaml; '' lets RealSense pick any camera."""
    path = Path(__file__).resolve().parent.parent / 'config' / 'camera_serials.yaml'
    try:
        serial = str(yaml.safe_load(path.read_text(encoding='utf-8'))['cameras']['base']['serial_no']).strip()
    except (OSError, KeyError, TypeError, yaml.YAMLError):
        return ''
    # The leading underscore keeps an all-digit SN a string for realsense2_camera.
    return '_' + serial if serial else ''


def generate_launch_description():
    base_serial = _base_camera_serial()
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='d435',
            namespace='camera',
            output='screen',
            parameters=[{
                **({'serial_no': base_serial} if base_serial else {}),
                'camera_name': 'd435',
                'base_frame_id': 'd435_link',
                'depth_frame_id': 'd435_depth_frame',
                'depth_optical_frame_id': 'd435_depth_optical_frame',
                'color_frame_id': 'd435_color_frame',
                'color_optical_frame_id': 'd435_color_optical_frame',
                'rgb_camera.color_profile': '1280x720x30',
                'depth_module.depth_profile': '848x480x30',
                'depth_module.infra_profile': '848x480x30',
                'pointcloud.enable': True,
                'pointcloud.ordered_pc': False,
                'align_depth.enable': True,
            }],
        )
    ])
