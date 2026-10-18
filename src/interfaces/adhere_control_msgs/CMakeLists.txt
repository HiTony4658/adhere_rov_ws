cmake_minimum_required(VERSION 3.5)
project(adhere_control_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# TODO(Phase 4+): populate msg/ and srv/ with the remaining real .msg / .srv definitions
# (TargetTrajectory.msg, KamrMppiTelemetry.msg, RhombicKernMetrics.msg, PadContactState.msg)
# once the 200Hz inner loop / 20Hz CUDA MPPI outer loop land.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FsmCommand.msg"
  "msg/SetpointCommand.msg"
  "msg/TeleopCommand.msg"
  "msg/ControllerState.msg"
  "msg/RuntimeDiagnostics.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
