from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import numpy as np

def generate_launch_description():
    realsense_share = FindPackageShare('realsense2_camera')
    navslam_share   = FindPackageShare('navslam')

    # config dosyamız
    default_config = PathJoinSubstitution([navslam_share, 'config', 'd456_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='RealSense parametre dosyası'
        ),

        #  base_link → camera_link sabit TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_link_tf',
            # Argümantlar: x y z roll pitch yaw parent child
            arguments=['0.6', '0', '0.45', '0', '0', '0', 'base_link', 'camera_camera_link'], 
            output='screen'
        ),

		#Node(
    	#	package='tf2_ros',
    	#	executable='static_transform_publisher',
    	#	name='base_to_imu_tf',
    	#	# Argümanlar: x y z roll pitch yaw parent child
    	#	# Burada x,y,z = sensörün robot üstündeki konumu
    	#	# roll pitch yaw = optical_frame → base_link dönüşümü
    	#	arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'base_link', 'camera_imu_optical_frame'],
    	#	output='screen'
		#),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([realsense_share, 'launch', 'rs_launch.py'])
            ),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file')
            }.items(),
        ),
    ])
