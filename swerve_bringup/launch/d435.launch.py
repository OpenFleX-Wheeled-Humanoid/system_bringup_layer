from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='d435',
            namespace='camera',
            output='screen',
            parameters=[{
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
