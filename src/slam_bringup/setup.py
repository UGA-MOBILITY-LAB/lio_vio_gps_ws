from setuptools import setup
import os
from glob import glob

package_name = 'slam_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='haohua',
    description='Launch files and configs for LIO-VIO-GPS fusion pipeline',
    license='MIT',
    entry_points={
        'console_scripts': [
            'carla_bridge = slam_bringup.carla_live_publisher:main',
        ],
    },
)
