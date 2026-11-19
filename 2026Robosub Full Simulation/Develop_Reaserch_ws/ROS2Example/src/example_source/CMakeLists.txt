cmake_minimum_required(VERSION 3.8)
project(example_source)

# 编译期打开常用警告,便于教学时发现问题
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖:ament_cmake 是构建系统本身,rclcpp/std_msgs 是运行库
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ---- 可执行程序 1:counter_node(源文件名可以和可执行名不同)----
add_executable(counter_node src/counter_node.cpp)
ament_target_dependencies(counter_node rclcpp std_msgs)

# ---- 可执行程序 2:feedback_node ----
add_executable(feedback_node src/feedback_node.cpp)
ament_target_dependencies(feedback_node rclcpp std_msgs)

# 安装到 install/example_source/lib/example_source/,
# `ros2 run example_source <名字>` 就是从这里找可执行文件。
install(TARGETS counter_node feedback_node
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 与 config 到 share 目录,供 ros2 launch 使用
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# 标准 ament 包注册:让 ament 索引能发现本包
ament_package()
