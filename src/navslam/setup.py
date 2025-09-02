import os
from glob import glob
from setuptools import setup

package_name = 'navslam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Tüm launch dosyalarını ekleme
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Tüm config YAML dosyalarını ekleme (alt klasörler dahil)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],

    zip_safe=True,
    maintainer='Batoddy',
    maintainer_email='batuhanodcikin@gmail.com',
    description='My ROS2 Python package example',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Çalıştırılabilir Python düğümlerini ekleme
            f'encoder_odom_bridge = {package_name}.encoder_esp32_bridge:main',
            f'motor_controller_esp32 = {package_name}.motor_controller_esp32:main'
        ],
    },
)
