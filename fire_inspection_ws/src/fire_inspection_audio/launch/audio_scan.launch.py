"""听觉检测节点 launch。

librosa 依赖 OpenSSL 3.0 legacy provider（部分版本），非交互终端不会读
~/.bashrc，这里显式设 CRYPTOGRAPHY_OPENSSL_NO_LEGACY=1 避免加载失败。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "dry_run",
            default_value="false",
            description="true=命中只在日志打印不发 alert（静默测试）",
        ),
    ]
    node = Node(
        package="fire_inspection_audio",
        executable="audio_scan_node",
        name="fire_audio_scan",
        output="screen",
        parameters=[{"dry_run": LaunchConfiguration("dry_run")}],
    )
    return LaunchDescription(
        [SetEnvironmentVariable("CRYPTOGRAPHY_OPENSSL_NO_LEGACY", "1")] + args + [node]
    )
