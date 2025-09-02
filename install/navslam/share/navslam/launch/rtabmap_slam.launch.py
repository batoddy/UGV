from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('navslam')
    remap_path = os.path.join(pkg, 'config', 'rtabmap_slam.yaml')

    mapping_arg = DeclareLaunchArgument(
        'mapping',
        default_value='false',
        description='Enable mapping (true) or localization-only (false)'
    )
    mapping_config = LaunchConfiguration('mapping')
    inverse_mapping_config = PythonExpression([ "'false' if '", mapping_config, "' == 'true' else 'true'" ])



    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            remap_path,
            {
                'Mem/IncrementalMemory': ParameterValue(mapping_config, value_type=str),
                'Mem/InitWMWithAllNodes': ParameterValue(inverse_mapping_config, value_type=str),
            }
        ],
        remappings=[
            ('/rgbd_image', '/rtabmap/rgbd_image'),
            ('/odom', '/odometry/filtered'),
        ],
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[
            remap_path,
            {
                'Mem/IncrementalMemory': ParameterValue(mapping_config, value_type=str),
                'Mem/InitWMWithAllNodes': ParameterValue(inverse_mapping_config, value_type=str),
				# 'RGBD/OptimizeMaxErrorRatio': 20.0
            }
        ],
        remappings=[
            ('/odom', '/rtabmap/odom'),
        ],
    )

    delayed_viz = TimerAction(period=1.0, actions=[rtabmap_viz])

    return LaunchDescription([
        mapping_arg,
        rtabmap_node,
        delayed_viz
    ])
