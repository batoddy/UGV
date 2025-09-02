from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('navslam')
    imu_madwick_yaml = os.path.join(pkg, 'config', 'imu_madwick.yaml')
    ekf_local_yaml = os.path.join(pkg, 'config', 'ekf_local.yaml')
    ekf_global_yaml = os.path.join(pkg, 'config', 'ekf_global.yaml')

    return LaunchDescription([
          
        # Madgwick Filter for IMU
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[imu_madwick_yaml],
            remappings=[
                ('/imu/data_raw', '/camera/camera/imu'),
                ('/imu/data', '/imu/filtered')
            ]
        ),
        # Local EKF (IMU + wheel + visual odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_local_yaml]
        ),

        # Global EKF (GPS + odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_global_yaml]
        ),

        # Navsat Transform Node (GPS -> odom)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[{
                'magnetic_declination_radians': 0.0,
                'yaw_offset': 0.0,
                'publish_filtered_gps': False
            }],
            remappings=[
                ('/gps/fix', '/pixhawk/fix'),
                ('/imu/data', '/pixhawk/imu'),
                ('/gps/filtered', '/odometry/gps')
            ]
        )

    ])
