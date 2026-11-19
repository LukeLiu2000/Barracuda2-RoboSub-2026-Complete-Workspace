from setuptools import setup
from glob import glob
import os

package_name = 'auv_bringup'

setup(
    name=package_name,
    version='1.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liuhao',
    maintainer_email='liuhao@example.com',
    description='RoboSub AUV 统一启动入口(控制器选择/目标源互斥/Foxglove 监测)',
    license='Apache-2.0',
)
