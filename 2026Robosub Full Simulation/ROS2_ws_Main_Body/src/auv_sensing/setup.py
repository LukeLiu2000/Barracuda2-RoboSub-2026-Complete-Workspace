from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'auv_sensing'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'isaac']),
    data_files=[
        # 标记文件 + 各类随包安装的资源(ament_python 需要 data_files 显式列出)
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'isaac'), glob('isaac/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liuhao',
    maintainer_email='liuhao@todo.todo',
    description='RoboSub AUV 感知包:ZED 仿真相机数据与 Isaac Sim 真值接入',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 注意:真值由 Isaac Sim 场景内的 ROS2PublishOdometry 图直接发布,
            # 不设空壳转发节点(见 README "设计取舍"一节)。
            'zed_sensor_node = auv_sensing.zed_sensor_node:main',
        ],
    },
)
