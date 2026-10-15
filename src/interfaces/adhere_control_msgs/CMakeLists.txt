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

# TODO(Phase 2): populate msg/ and srv/ with the real .msg / .srv definitions
# (FsmCommand.msg, TargetPose.msg, TargetTrajectory.msg, KamrMppiTelemetry.msg,
#  RhombicKernMetrics.msg, PadContactState.msg, SwitchFsmState.srv) and list
# them here, e.g.:
#
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/FsmCommand.msg"
#   "msg/TargetPose.msg"
#   "msg/TargetTrajectory.msg"
#   "msg/KamrMppiTelemetry.msg"
#   "msg/RhombicKernMetrics.msg"
#   "msg/PadContactState.msg"
#   "srv/SwitchFsmState.srv"
#   DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
