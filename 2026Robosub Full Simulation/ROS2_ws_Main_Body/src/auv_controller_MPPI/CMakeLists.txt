cmake_minimum_required(VERSION 3.16)
project(auv_controller_MPPI)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
# OpenMP:并行 MPPI rollout(可选 —— 未找到则单线程编译,算法不变)
find_package(OpenMP QUIET)

# ===========================================================================
# 核心算法库:纯 C++17 + Eigen,不依赖 ROS —— 便于单元测试和未来迁移 CUDA。
# mppi_solver 在 CPU 上单线程滚动 K 条 rollout;rollout 内核是纯函数,
# 与 ROS 完全解耦(见 include/auv_controller_mppi/ 下各头文件说明)。
# ===========================================================================
add_library(mppi_core STATIC
  src/thruster_model.cpp
  src/dynamics_model.cpp
  src/cost_function.cpp
  src/mppi_solver.cpp
  src/control_sequence.cpp
  src/sequence_executor.cpp
)
target_include_directories(mppi_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})
if(OPENMP_FOUND)
  target_link_libraries(mppi_core PUBLIC OpenMP::OpenMP_CXX)
  message(STATUS "OpenMP enabled for MPPI rollouts")
endif()

# ===========================================================================
# ROS 2 节点
# ===========================================================================
function(add_auv_node name)
  if(ARGV1)
    add_executable(${name} ${ARGV1})
  else()
    add_executable(${name} src/${name}.cpp)
  endif()
  target_include_directories(${name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(${name} mppi_core)
  ament_target_dependencies(${name}
    rclcpp std_msgs nav_msgs geometry_msgs diagnostic_msgs std_srvs)
  install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
endfunction()

add_auv_node(mppi_controller_node)         # 常规模式 MPPI 控制器
add_auv_node(training_mission_node)        # 训练任务生成 / 激励实验
add_auv_node(training_data_collector_node) # 状态-控制-下一状态 记录器
add_auv_node(basic_mission_test src/basic_mission.cpp)  # 无通信闭环基础任务测试(第十二章)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

# ===========================================================================
# 单元 / 集成测试(gtest)
# ===========================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 纯算法测试(不依赖 ROS 运行时)
  ament_add_gtest(test_thruster_model test/test_thruster_model.cpp)
  target_link_libraries(test_thruster_model mppi_core)

  ament_add_gtest(test_dynamics_model test/test_dynamics_model.cpp)
  target_link_libraries(test_dynamics_model mppi_core)

  ament_add_gtest(test_mppi_solver test/test_mppi_solver.cpp)
  target_link_libraries(test_mppi_solver mppi_core)

  ament_add_gtest(test_cost_function test/test_cost_function.cpp)
  target_link_libraries(test_cost_function mppi_core)

  ament_add_gtest(test_control_sequence test/test_control_sequence.cpp)
  target_link_libraries(test_control_sequence mppi_core)

  # ROS 2 接口集成测试(发布 odom + goal,断言 thruster_cmd 输出)
  ament_add_gtest(test_ros_interface test/test_ros_interface.cpp)
  target_link_libraries(test_ros_interface mppi_core)
  ament_target_dependencies(test_ros_interface
    rclcpp std_msgs nav_msgs geometry_msgs diagnostic_msgs std_srvs)
endif()

ament_package()
