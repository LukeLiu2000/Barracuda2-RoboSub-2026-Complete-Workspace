from setuptools import setup
from glob import glob
import os

package_name = 'auv_controller_gru'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name,
              package_name + '.env',
              package_name + '.train'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liuhao',
    maintainer_email='liuhao@example.com',
    description='RoboSub AUV GRU 控制器(推理 + 训练)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gru_controller_node = auv_controller_gru.gru_controller_node:main',
        ],
    },
    tests_require=['pytest'],
)
