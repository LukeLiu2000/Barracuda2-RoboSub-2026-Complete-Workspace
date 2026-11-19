cmake_minimum_required(VERSION 3.8)
project(auv_isaac_bridge)

find_package(ament_cmake REQUIRED)

# isaac/ 与 config/ 按源码树路径使用(bootstrap.py 里写的是源码树绝对路径),
# 不装进 install 空间;scripts 装进 lib 以便 ros2 run / launch 使用。
install(DIRECTORY config isaac launch
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/bridge_monitor.py scripts/bridge_client.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
