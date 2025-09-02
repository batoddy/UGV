from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('navslam')
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # map_server = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{'yaml_filename': '/home/orinium/.ros/rtabmap.yaml'}],
    #     remappings=[
    #         ('map', '/main_map'),
    #         ('map_metadata', '/main_map_metadata')
    #     ]
    # )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params]
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params]
    )

    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params]
    )

    bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params]
    )

    waypoint = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params]
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 0.0,
            'use_sim_time': False,
            'node_names': [
        #         'map_server',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )

    return LaunchDescription([
        # map_server,      # <<--- EKLENDİ
        controller,
        planner,
        behavior,
        bt_nav,
        waypoint,
        lifecycle
    ])
