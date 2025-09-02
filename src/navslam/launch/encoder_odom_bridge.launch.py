from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('navslam')  # kendi paketin adı

    return LaunchDescription([
        Node(
            package='navslam',            # senin package.xml’de tanımlı paket ismi
            executable='encoder_odom_bridge',  # setup.py’de entry_point olarak verdiğin isim
            name='encoder_odom_bridge',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',  # istersen parametre yapabilirsin
                'baud': 115200
            }]
        )
    ])
