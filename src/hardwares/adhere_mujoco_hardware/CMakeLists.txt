cmake_minimum_required(VERSION 3.8)
project(adhere_mujoco_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(adhere_controller_common REQUIRED)

include_directories(include)

# TODO(Phase 2): find MuJoCo (libmujoco), GLFW3, and OpenGL once vendored, e.g.:
# find_package(mujoco REQUIRED)
# find_package(glfw3 REQUIRED)
# find_package(OpenGL REQUIRED)

# TODO(Phase 2): add real sources once implemented.
# add_library(${PROJECT_NAME} SHARED
#   src/adhere_mujoco_hardware.cpp
#   src/mujoco_vis_bridge.cpp
#   src/mujoco_viewer.cpp
# )
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# target_link_libraries(${PROJECT_NAME}
#   adhere_controller_common::adhere_controller_common
#   mujoco::mujoco
#   glfw
#   OpenGL::GL
# )
# ament_target_dependencies(${PROJECT_NAME}
#   ament_index_cpp
#   rclcpp
#   rclcpp_lifecycle
#   hardware_interface
#   controller_manager
#   pluginlib
#   realtime_tools
# )
# pluginlib_export_plugin_description_file(hardware_interface adhere_mujoco_hardware.xml)
#
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES adhere_mujoco_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
