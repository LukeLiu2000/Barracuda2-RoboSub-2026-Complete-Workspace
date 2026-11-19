from setuptools import setup
from glob import glob
import os

package_name = 'auv_experiments'

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
    description='RoboSub 实验/评测公共包(任务/编排/记录/指标/绘图/整定/报告)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'experiment_runner = '
            'auv_experiments.experiment_runner:main',
            'analyze = auv_experiments.analyze:main',
            'tune_pid = auv_experiments.tune_pid:main',
            'make_full_report = auv_experiments.make_full_report:main',
        ],
    },
    tests_require=['pytest'],
)
