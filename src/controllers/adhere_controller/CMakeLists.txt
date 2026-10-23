cmake_minimum_required(VERSION 3.8)
project(adhere_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # CLAUDE.md Phase 4.1: scoped to CXX only via COMPILE_LANGUAGE -- nvcc (the CUDA compiler
  # enabled further below for baseline_mppi_kernels.cu) does not understand -Wall/-Wextra/
  # -Wpedantic directly and would need them wrapped in -Xcompiler.
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
  )
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

# Pure-CUDA Baseline MPPI (CLAUDE.md Phase 4.1): CMAKE_CUDA_ARCHITECTURES matches Sec.2's target
# hardware exactly (Jetson AGX Xavier sm_72 + RTX5000 workstation sm_75) -- do not widen this to
# include newer architectures the project does not target. nvcc is not on this workstation's
# default $PATH (mandated build command only prepends /usr/bin, CLAUDE.md Sec.0), so fall back to
# the known CUDA 11.4 toolkit location if CMAKE_CUDA_COMPILER was not already set externally.
if(NOT DEFINED CMAKE_CUDA_COMPILER AND EXISTS "/usr/local/cuda/bin/nvcc")
  set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
endif()
enable_language(CUDA)
set(CMAKE_CUDA_ARCHITECTURES "72;75")
if(NOT CMAKE_CUDA_STANDARD)
  # CMake 3.16 on this workstation does not recognize CUDA_STANDARD 17 (added in later CMake);
  # CUDA 11.4's nvcc itself supports C++17, but the project-wide language standard is left at 14
  # here to keep cmake_minimum_required(3.8) buildable on this host.
  set(CMAKE_CUDA_STANDARD 14)
endif()
# CMake 3.16 on this workstation predates FindCUDAToolkit (needs 3.17+) -- the legacy FindCUDA
# module still works fine alongside enable_language(CUDA) above and provides
# CUDA_INCLUDE_DIRS/CUDA_LIBRARIES (cudart)/CUDA_curand_LIBRARY for the plain-g++-compiled host
# .cpp files (BaselineMppiController.cpp) that include <cuda_runtime.h> directly.
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3 --use_fast_math")
find_package(CUDA REQUIRED)

# TODO(Phase 3.2+/KAMR-MPPI): find osqp / osqp-eigen once the 12D Elastic Rhombic-Kern QP is
# implemented.
# find_package(osqp REQUIRED)
# find_package(OsqpEigen REQUIRED)

# Single unified ControllerInterface plugin (CLAUDE.md Phase 3.3): FSM lifecycle management
# (adhere_controller.cpp) + all FSMState implementations (StatePassive/StateOpenLoop/
# StatePidHold/StateAdhere*) in one library/package -- these never build or deploy
# independently of each other, so the former adhere_controller_common/adhere_kamr_controller
# package split added pure linking overhead with no decoupling benefit.
add_library(${PROJECT_NAME} SHARED
  src/adhere_controller.cpp
  src/algorithms/baseline_mppi/BaselineMppiConfig.cpp
  src/algorithms/baseline_mppi/BaselineMppiController.cpp
  src/algorithms/baseline_mppi/baseline_mppi_kernels.cu
  src/algorithms/pid/AdherePidController.cpp
  src/algorithms/pid/SixDofPidController.cpp
  src/algorithms/smc/SixDofSmcController.cpp
  src/allocation/ThrusterAllocator.cpp
  src/dynamics/FossenDynamics.cpp
  src/dynamics/ModelParams.cpp
  src/estimator/RovStateEstimator.cpp
  src/FSM/StatePassive.cpp
  src/FSM/StateAdhereBase.cpp
  src/FSM/StateAdherePid.cpp
  src/FSM/StateAdhereSmc.cpp
  src/FSM/StateBaselineMppi.cpp
  src/FSM/StateOpenLoop.cpp
  src/FSM/StatePidHold.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES} ${CUDA_curand_LIBRARY})
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
  target_link_libraries(test_thruster_allocator ${PROJECT_NAME} Eigen3::Eigen)
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
  # closed-loop StatePidHold tracking driven entirely through "sensor_fusion" (never ground
  # truth), and the Xbox LT/RT -> pitch/roll teleop mapping.
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

  # Phase 3.7 verification: FossenDynamics (M symmetry/PD, skew-symmetric Coriolis, dissipative
  # damping, exact buoyancy restoring force), SixDofPidController (quaternion hemisphere
  # invariance, conditional anti-windup), RovStateEstimator's contact-aided wall-constraint
  # update, and the common/OutputSafety.h NaN/Inf sanitization gate.
  ament_add_gtest(test_dynamics_and_pid
    test/test_dynamics_and_pid.cpp
  )
  target_include_directories(test_dynamics_and_pid PRIVATE include)
  target_link_libraries(test_dynamics_and_pid ${PROJECT_NAME} Eigen3::Eigen)
  ament_target_dependencies(test_dynamics_and_pid rclcpp hardware_interface)

  # Phase 3.8 verification: StateAdherePid/StateAdhereSmc closed-loop wall-docking contract
  # (4-pad contact established, positive Rhombic Kern margin, Y/Z/attitude hold) against a fake
  # one-sided wall-contact spring plant, plus the comparative PID-vs-SMC disturbance-rejection/
  # chattering trade-off check.
  ament_add_gtest(test_adhere_wall_states
    test/test_adhere_wall_states.cpp
  )
  target_include_directories(test_adhere_wall_states PRIVATE include)
  target_link_libraries(test_adhere_wall_states ${PROJECT_NAME})
  ament_target_dependencies(test_adhere_wall_states rclcpp hardware_interface)

  # Header-only (ControllerMetrics.h): no rclcpp/hardware_interface dependency, so the
  # efficiency/chattering/safety telemetry contract (CLAUDE.md Phase 3.8 Stage 3.1) is
  # verifiable in isolation.
  ament_add_gtest(test_controller_metrics
    test/test_controller_metrics.cpp
  )
  target_include_directories(test_controller_metrics PRIVATE include)
  target_link_libraries(test_controller_metrics ${PROJECT_NAME})

  # Phase 4.1 verification: exercises the real CUDA kernels on this workstation's GPU (no mock) --
  # determinism/latency/thruster-bound safety, directional cost-gradient sanity, and closed-loop
  # position tracking through StateBaselineMppi's 250Hz ZOH against a fake point-mass plant (same
  # idiom as test_fsm_states.cpp).
  ament_add_gtest(test_baseline_mppi
    test/test_baseline_mppi.cpp
  )
  target_include_directories(test_baseline_mppi PRIVATE include)
  target_link_libraries(test_baseline_mppi ${PROJECT_NAME} Eigen3::Eigen)
  ament_target_dependencies(test_baseline_mppi rclcpp hardware_interface)
endif()

ament_package()
