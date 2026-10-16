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
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------------------
# MuJoCo discovery. No official find_package(mujoco) CMake-config is assumed to be
# installed; search the usual locations in priority order (CLAUDE.md Sec.2/3):
#   1. $MUJOCO_DIR environment variable (explicit override)
#   2. ~/.mujoco/mujoco-* (the common per-user install location used on this workstation)
#   3. /opt/mujoco
#   4. /usr/local
# ---------------------------------------------------------------------------------------
file(GLOB MUJOCO_HOME_CANDIDATES "$ENV{HOME}/.mujoco/mujoco-*")
find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  HINTS $ENV{MUJOCO_DIR} ${MUJOCO_HOME_CANDIDATES} /opt/mujoco /usr/local
  PATH_SUFFIXES include
)
find_library(MUJOCO_LIBRARY
  NAMES mujoco
  HINTS $ENV{MUJOCO_DIR} ${MUJOCO_HOME_CANDIDATES} /opt/mujoco /usr/local
  PATH_SUFFIXES lib bin
)
if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR
    "MuJoCo not found. Set $MUJOCO_DIR to the MuJoCo install root "
    "(e.g. ~/.mujoco/mujoco-3.x.x) or install to /opt/mujoco.")
endif()
message(STATUS "Found MuJoCo include: ${MUJOCO_INCLUDE_DIR}")
message(STATUS "Found MuJoCo library: ${MUJOCO_LIBRARY}")

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW3 REQUIRED glfw3)
find_package(OpenGL REQUIRED)

include_directories(include ${MUJOCO_INCLUDE_DIR})

add_library(${PROJECT_NAME} SHARED
  src/adhere_mujoco_hardware.cpp
  src/fossen_hydrodynamics.cpp
  src/flow_environment_plugin.cpp
  src/mujoco_viewer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
  ${GLFW3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}
  ${MUJOCO_LIBRARY}
  ${GLFW3_LIBRARIES}
  OpenGL::GL
)
target_link_directories(${PROJECT_NAME} PUBLIC ${GLFW3_LIBRARY_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_manager
  pluginlib
  realtime_tools
  adhere_controller_common
)
pluginlib_export_plugin_description_file(hardware_interface adhere_mujoco_hardware.xml)

add_executable(adhere_mujoco_node src/adhere_mujoco_node.cpp)
target_include_directories(adhere_mujoco_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(adhere_mujoco_node ${PROJECT_NAME})
ament_target_dependencies(adhere_mujoco_node
  rclcpp
  hardware_interface
  controller_manager
  tf2_ros
  nav_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS adhere_mujoco_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES adhere_mujoco_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fossen_hydrodynamics_geometry
    test/test_fossen_hydrodynamics_geometry.cpp
    src/fossen_hydrodynamics.cpp
  )
  target_include_directories(test_fossen_hydrodynamics_geometry PRIVATE
    include ${MUJOCO_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(test_fossen_hydrodynamics_geometry ${MUJOCO_LIBRARY})
  ament_target_dependencies(test_fossen_hydrodynamics_geometry ament_index_cpp)

  # Header-only (adhere_mujoco_hardware/rhombic_kern.hpp): no MuJoCo/mjModel/ResourceManager
  # dependency, so this target needs neither MUJOCO_LIBRARY nor ament_index_cpp -- the
  # hydrodynamic-accuracy audit's "no test coverage for the 500Hz ESC/Rhombic-Kern logic"
  # finding, addressed by factoring that logic out into pure, independently-testable functions.
  ament_add_gtest(test_rhombic_kern_and_thruster_model
    test/test_rhombic_kern_and_thruster_model.cpp
  )
  target_include_directories(test_rhombic_kern_and_thruster_model PRIVATE include)
endif()

ament_package()
