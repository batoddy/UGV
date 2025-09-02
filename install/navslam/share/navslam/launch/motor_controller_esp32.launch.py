from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Parametreleri terminalden kolayca değiştirmek için argümanlar tanımlama
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Seri port adı.'
    )
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Seri portun baud hızı.'
    )
    track_width_arg = DeclareLaunchArgument(
        'track_width',
        default_value='0.50',
        description='Robotun palet genişliği (metre).'
    )
    max_lin_arg = DeclareLaunchArgument(
        'max_lin',
        default_value='1.0',
        description='Maksimum doğrusal hız (m/s).'
    )
    max_ang_arg = DeclareLaunchArgument(
        'max_ang',
        default_value='2.0',
        description='Maksimum açısal hız (rad/s).'
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        track_width_arg,
        max_lin_arg,
        max_ang_arg,

        Node(
            package='navslam', # Kodunuzun bulunduğu ROS 2 paket adı
            executable='motor_controller_esp32', # Python dosyasının adı
            name='motor_controller_esp32',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'track_width': LaunchConfiguration('track_width'),
                'max_lin': LaunchConfiguration('max_lin'),
                'max_ang': LaunchConfiguration('max_ang'),
                'rate_limit': 2.0,
                'deadzone': 0.01,
            }]
        )
    ])
'''
eof

### Kullanım

1.  **Dosyaları Kaydedin:**
    * Python kodunuzu `twist_to_serial.py` olarak `~/orn_ws/src/navslam/launch/` dizinine kaydedin.
    * Yukarıdaki launch dosyasını `twist_to_serial.launch.py` olarak `~/orn_ws/src/navslam/launch/` dizinine kaydedin.

2.  **Çalıştırılabilir Yapın:**
    Terminalde Python dosyanızı çalıştırılabilir yapın:
    ```bash
    chmod +x ~/orn_ws/src/navslam/launch/twist_to_serial.py
    ```

3.  **Başlatma:**
    Artık launch dosyasını kullanabilirsiniz.
    ```bash
    ros2 launch navslam twist_to_serial.launch.py
    
'''