cmake_minimum_required(VERSION 3.8)
project(adhere_controller)

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
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(adhere_control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

# TODO(Phase 2): enable CUDA language once outer_mppi_cuda/*.cu kernels are authored.
# enable_language(CUDA)
# set(CMAKE_CUDA_ARCHITECTURES "72;75")
# find_package(CUDAToolkit REQUIRED)

# TODO(Phase 3.2+/KAMR-MPPI): find osqp / osqp-eigen once the 12D Elastic Rhombic-Kern QP is
# implemented.
# find_package(osqp REQUIRED)
# find_package(OsqpEigen REQUIRED)

# Single unified ControllerInterface plugin (CLAUDE.md Phase 3.3): FSM lifecycle management
# (adhere_controller.cpp) + all FSMState implementations (StatePassive/StateDepthHold/
# StateOpenLoop/StatePidHold) in one library/package -- these never build or deploy
# independently of each other, so the former adhere_controller_common/adhere_kamr_controller
# package split added pure linking overhead with no decoupling benefit.
add_library(${PROJECT_NAME} SHARED
  src/adhere_controller.cpp
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
  pluginlib
  realtime_tools
  adhere_control_msgs
  visualization_msgs
)
pluginlib_export_plugin_description_file(controller_interface adhere_controller.xml)

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
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES adhere_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_interface
  pluginlib
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

  # Header-only (ModelParams.h + ThrusterAllocator.h): pure-Eigen allocation-matrix contract
  # (CLAUDE.md Phase 3.2/3.3 Stage 3.2) -- B*B^dagger ~= I_6 and zero parasitic pitch/yaw
  # coupling for pure sway/heave, built dynamically from a ModelParams fixture instead of
  # hardcoded corner geometry.
  ament_add_gtest(test_thruster_allocator
    test/test_thruster_allocator.cpp
  )
  target_include_directories(test_thruster_allocator PRIVATE include)
  target_link_libraries(test_thruster_allocator Eigen3::Eigen)
  ament_target_dependencies(test_thruster_allocator rclcpp)

  # Closed-loop FSM state contract (CLAUDE.md Phase 3.2/3.3 Stage 3.2): fakes CtrlInterfaces
  # with hardware_interface::StateInterface/CommandInterface backed by local doubles (no
  # ResourceManager needed), and drives each state's run() against a simple point-mass/attitude
  # Euler-integration "plant" owned by the test itself.
  ament_add_gtest(test_fsm_states
    test/test_fsm_states.cpp
  )
  target_include_directories(test_fsm_states PRIVATE include)
  target_link_libraries(test_fsm_states ${PROJECT_NAME})
  ament_target_dependencies(test_fsm_states rclcpp hardware_interface)

  # Phase 3.5 verification (CLAUDE.md Stage 3): RovStateEstimator's DVL lever-arm compensation,
  # closed-loop StateDepthHold/StatePidHold tracking driven entirely through "sensor_fusion"
  # (never ground truth), and the Xbox LT/RT -> pitch/roll teleop mapping.
  ament_add_gtest(test_sensor_fusion_and_teleop
    test/test_sensor_fusion_and_teleop.cpp
  )
  target_include_directories(test_sensor_fusion_and_teleop PRIVATE include)
  target_link_libraries(test_sensor_fusion_and_teleop ${PROJECT_NAME})
  ament_target_dependencies(test_sensor_fusion_and_teleop rclcpp hardware_interface)

  # Header-only (LoopProfiler.h): no rclcpp/hardware_interface dependency, so the Hz/latency/
  # overrun instrumentation contract (CLAUDE.md Phase 3.6 Stage 3.2) is verifiable in isolation.
  ament_add_gtest(test_loop_profiler
    test/test_loop_profiler.cpp
  )
  target_include_directories(test_loop_profiler PRIVATE include)
endif()

ament_package()
