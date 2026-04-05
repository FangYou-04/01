from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取功能包路径
    pkg_dir = get_package_share_directory('armor_pose_solver')
    # 测试视频路径（放在 config 目录下）
    video_path = os.path.join(pkg_dir, 'config', '5.mp4')

    return LaunchDescription([
        Node(
            package='armor_pose_solver',
            executable='armor_pose_node',
            name='armor_pose_node',
            output='screen',
            parameters=[
                {'video_path': video_path}  # 可通过参数传递路径，后续代码可读取
            ]
        )
    ])
