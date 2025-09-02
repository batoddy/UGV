from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    pkg = get_package_share_directory('navslam')

    base_delay = 2.5  # saniye

    mapping_arg = DeclareLaunchArgument(
        'mapping',
        default_value='false',
        description='Enable mapping (true) or localization-only (false)'
    )

    mapping_config = LaunchConfiguration('mapping')

    def delayed_include(offset_sec: float, filename: str, extra_args=None):
        args = extra_args if extra_args else {}
        return TimerAction(
            period=offset_sec,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg, 'launch', filename)
                    ),
                    launch_arguments=[(k, v) for k, v in args.items()]
                )
            ]
        )

    return LaunchDescription([
        mapping_arg,
        GroupAction([
            PushRosNamespace(''),

            delayed_include(0.0 * base_delay, 'd456_launch.launch.py'),
            delayed_include(1.0 * base_delay, 'rgbd_odom.launch.py'),
			delayed_include(1.0 * base_delay, 'encoder_odom_bridge.launch.py'),

            delayed_include(2.0 * base_delay, 'ekf_fusion.launch.py'),
            # mapping paramını alt launch’a geçiyoruz
            delayed_include(3.0 * base_delay, 'rtabmap_slam.launch.py',
                            extra_args={'mapping': mapping_config}),

            delayed_include(5.0 * base_delay, 'nav2_navigation.launch.py'),
        ])
    ])
