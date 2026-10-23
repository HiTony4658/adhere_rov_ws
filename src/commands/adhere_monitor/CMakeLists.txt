cmake_minimum_required(VERSION 3.8)
project(adhere_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(adhere_control_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rt_monitor_node src/rt_monitor_node.cpp)
ament_target_dependencies(rt_monitor_node
  rclcpp
  adhere_control_msgs
)

add_executable(benchmark_evaluator_node src/benchmark_evaluator_node.cpp)
ament_target_dependencies(benchmark_evaluator_node
  rclcpp
  adhere_control_msgs
  nav_msgs
  std_msgs
)

install(TARGETS rt_monitor_node benchmark_evaluator_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
