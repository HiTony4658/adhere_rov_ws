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
find_package(nav_msgs REQUIRED)

include_directories(include)

# TODO(Phase 3.2+): add the benchmark-trigger/teleop executable once station-keeping and
# trajectory-tracking targets + gust injection are implemented.
add_executable(fsm_console_node src/fsm_console_node.cpp)
target_include_directories(fsm_console_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fsm_console_node
  rclcpp
  adhere_control_msgs
  nav_msgs
)
install(TARGETS fsm_console_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
