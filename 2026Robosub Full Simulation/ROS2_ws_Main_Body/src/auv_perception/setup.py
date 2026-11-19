from setuptools import setup, find_packages
from glob import glob
setup(name='auv_perception', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',['resource/auv_perception']),
                  ('share/auv_perception',['package.xml']),
                  ('share/auv_perception/config',glob('config/*.yaml')),
                  ('share/auv_perception/config/templates',glob('config/templates/*.yaml')),
                  ('share/auv_perception/launch',glob('launch/*.py')),
                  ('share/auv_perception/docs',glob('docs/*')),
                  ('share/auv_perception/scripts',glob('scripts/*'))],
      entry_points={'console_scripts': ['perception_node=auv_perception.node:main',
                                        'synced_zed_node=auv_perception.synced_sensing:main',
                                        'scene_node=auv_perception.scene:main']},
      install_requires=['setuptools'], description='Semantic rigid object perception and world landmarks',
      maintainer='liuhao',maintainer_email='liuhao@localhost',license='MIT')
