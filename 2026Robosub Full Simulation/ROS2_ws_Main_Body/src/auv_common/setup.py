from setuptools import setup
from glob import glob
import os

package_name = 'auv_common'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liuhao',
    maintainer_email='liuhao@example.com',
    description='AUV 公共配置与几何/分配/坐标系工具库',
    license='Apache-2.0',
    tests_require=['pytest'],
)
