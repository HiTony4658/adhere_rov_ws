cmake_minimum_required(VERSION 3.8)
project(adhere_keyboard_input)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(adhere_control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(fsm_console_node src/fsm_console_node.cpp)
target_include_directories(fsm_console_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fsm_console_node
  rclcpp
  adhere_control_msgs
  geometry_msgs
)

add_executable(state_cmd_console_node src/state_cmd_console_node.cpp)
target_include_directories(state_cmd_console_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(state_cmd_console_node
  rclcpp
  adhere_control_msgs
  geometry_msgs
)

add_executable(xbox_teleop_node src/xbox_teleop_node.cpp)
target_include_directories(xbox_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(xbox_teleop_node
  rclcpp
  adhere_control_msgs
  geometry_msgs
  sensor_msgs
)

install(TARGETS fsm_console_node state_cmd_console_node xbox_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
