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

# TODO(Phase 3.2+): populate msg/ and srv/ with the remaining real .msg / .srv definitions
# (TargetPose.msg, TargetTrajectory.msg, KamrMppiTelemetry.msg, RhombicKernMetrics.msg,
#  PadContactState.msg, SwitchFsmState.srv) and list them here alongside FsmCommand.msg.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FsmCommand.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
