cmake_minimum_required(VERSION 3.8)
project(adhere_controller_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(adhere_control_msgs REQUIRED)

include_directories(include)

# TODO(Phase 4+): add FossenHydrodynamics relocation once the 200Hz inner loop / 20Hz CUDA MPPI
# outer loop land; StatePassive/StateDepthHold/StateOpenLoop/StatePidHold are the Phase 3.2 FSM
# states.
add_library(${PROJECT_NAME} SHARED
  src/FSM/StatePassive.cpp
  src/FSM/StateDepthHold.cpp
  src/FSM/StateOpenLoop.cpp
  src/FSM/StatePidHold.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_interface
  realtime_tools
  adhere_control_msgs
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_interface
  realtime_tools
  adhere_control_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # Header-only (MultiRateDecimator.h): no rclcpp/hardware_interface dependency, so the uniform
  # 500Hz->250Hz/25Hz decimation contract (CLAUDE.md Phase 3.1.1 Stage 3.2) is verifiable in
  # isolation, without standing up a full ControllerInterface/ResourceManager.
  ament_add_gtest(test_multirate_decimator
    test/test_multirate_decimator.cpp
  )
  target_include_directories(test_multirate_decimator PRIVATE include)

  # Header-only (ThrusterAllocator.h): pure-Eigen allocation-matrix contract (CLAUDE.md Phase
  # 3.2 Stage 3.2) -- B*B^dagger ~= I_6 and zero parasitic pitch/yaw coupling for pure sway/heave.
  ament_add_gtest(test_thruster_allocator
    test/test_thruster_allocator.cpp
  )
  target_include_directories(test_thruster_allocator PRIVATE include)
  target_link_libraries(test_thruster_allocator Eigen3::Eigen)

  # Closed-loop FSM state contract (CLAUDE.md Phase 3.2 Stage 3.2): fakes CtrlInterfaces with
  # hardware_interface::StateInterface/CommandInterface backed by local doubles (no
  # ResourceManager needed), and drives each state's run() against a simple point-mass/attitude
  # Euler-integration "plant" owned by the test itself.
  ament_add_gtest(test_fsm_states
    test/test_fsm_states.cpp
  )
  target_include_directories(test_fsm_states PRIVATE include)
  target_link_libraries(test_fsm_states ${PROJECT_NAME})
  ament_target_dependencies(test_fsm_states rclcpp hardware_interface)
endif()

ament_package()
