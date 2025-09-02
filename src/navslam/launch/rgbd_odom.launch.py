# launch/rgbd_odom.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    pkg = get_package_share_directory('navslam')
    remap_path = os.path.join(pkg, 'config', 'camera_remaps.yaml')

    with open(remap_path, 'r') as f:
        remaps = yaml.safe_load(f)['camera_remaps']

    # YAML’dan okunanlar:
    rgb = remaps['rgb_image']
    depth = remaps['depth_image']
    info = remaps['camera_info']
    imu  = remaps['imu']

    return LaunchDescription([
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            namespace='rtabmap',
            name='rtabmap_sync',
            output='screen',
            remappings=[
                ('rgb/image',        rgb),
                ('depth/image',      depth),
                ('rgb/camera_info',  info),
                ('rgbd_image',       'rgbd_image')
            ],
            parameters=[{'approx_sync': False,
			'queue_size': 30,
			'qos_image': 1,
			'qos_camera_info': 1
			}]
        ),

        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            namespace='rtabmap',
            output='screen',
            remappings=[
                ('rgbd_image', 'rgbd_image'),
                ('imu',        imu)  ,
            ],
            parameters=[{
                'Odom/ResetCountdown': '0',
                'Odom/FillInfoData': "true",
				'subscribe_rgbd': True ,
                'wait_imu_to_init': False, # IMU kullanmak için True yap - Orientation eklenince aç
				'publish_tf': False
            }]
        )
    ])