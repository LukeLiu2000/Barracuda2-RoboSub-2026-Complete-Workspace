# setup.py —— ament_python 包的"构建脚本"(setuptools 标准)。
# colcon build 对本包执行的就是安装(pip 风格),没有 CMake 编译这一步。

from setuptools import setup

package_name = 'example_processing'

setup(
    name=package_name,
    version='0.1.0',
    description='ROS 2 教学示例 Python 包:square_node / report_node',
    maintainer='liuhao',
    maintainer_email='liuhao@example.com',
    license='Apache-2.0',

    # 内层同名目录 example_processing/ 是真正的 Python 包
    packages=[package_name],

    # 额外安装的非 Python 文件:资源标记、package.xml、launch、config
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/processing.launch.py']),
        ('share/' + package_name + '/config',
            ['config/processing.yaml']),
    ],

    zip_safe=True,

    # colcon test 依据此声明选择 pytest 运行器(否则退回 unittest 发现,
    # Python 3.12 下无 __init__.py 的 test/ 目录会收集到 0 个测试)
    tests_require=['pytest'],

    # console_scripts:注册两个命令行入口,安装后
    # `ros2 run example_processing square_node` 就能找到它们
    entry_points={
        'console_scripts': [
            'square_node = example_processing.square_node:main',
            'report_node = example_processing.report_node:main',
        ],
    },
)
