"""FT Framework ROS2 package setup."""

from setuptools import setup
import os
from glob import glob

package_name = 'ft_framework'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhengyuan.liu',
    maintainer_email='zhengyuan.liu@example.com',
    description='FT Radar-Camera-Vehicle Data Fusion Framework for ROS2 Humble',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'adc_rx = ft_framework.adc_rx:main',
            'camera_rx = ft_framework.camera_rx:main',
            'vehicle_data_rx = ft_framework.vehicle_data_rx:main',
            'rsp_mil_python = ft_framework.rsp_mil_python:main',
            'rsp_cuda = ft_framework.rsp_cuda:main',
            'rviz_radar = ft_framework.rviz_radar:main',
            'rviz_image = ft_framework.rviz_image:main',
            'logging_node = ft_framework.logging_node:main',
            'object_detection_3d = ft_framework.object_detection_3d:main',
            'rviz_ruler = ft_framework.rviz_ruler:main',
        ],
    },
)
