"""FT Framework ROS2 package setup (V2 架构)."""

from setuptools import setup
import os
from glob import glob

package_name = 'ft_framework'

setup(
    name=package_name,
    version='2.0.0',
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
    description='FT Radar-Camera-Vehicle Data Fusion Framework V2 (ROS2 Foxy, Jetson AGX Orin)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # V2: 仅保留 GPU RSP + 可视化 + 监控 + 感知
            # 数据采集层已迁移到 ft_rx_cpp (C++)
            # logging_node 已内置到 C++ rx 节点
            # rsp_mil_python 已移除 (仅使用 GPU 版本)
            'rsp_cuda = ft_framework.rsp_cuda:main',
            'rviz_radar = ft_framework.rviz_radar:main',
            'rviz_image = ft_framework.rviz_image:main',
            'rviz_ruler = ft_framework.rviz_ruler:main',
            'object_detection_3d = ft_framework.object_detection_3d:main',
            'system_monitor = ft_framework.system_monitor:main',
        ],
    },
)
