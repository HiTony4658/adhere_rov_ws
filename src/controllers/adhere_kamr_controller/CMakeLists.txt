cmake_minimum_required(VERSION 3.8)
project(adhere_kamr_controller)

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
find_package(adhere_controller_common REQUIRED)
find_package(adhere_control_msgs REQUIRED)

include_directories(include)

# TODO(Phase 2): enable CUDA language once outer_mppi_cuda/*.cu kernels are authored.
# enable_language(CUDA)
# set(CMAKE_CUDA_ARCHITECTURES "72;75")
# find_package(CUDAToolkit REQUIRED)

# TODO(Phase 2): find osqp / osqp-eigen once the 12D Elastic Rhombic-Kern QP is implemented.
# find_package(osqp REQUIRED)
# find_package(OsqpEigen REQUIRED)

# TODO(Phase 2): add real sources once implemented, e.g.:
# add_library(${PROJECT_NAME} SHARED
#   src/adhere_kamr_controller.cpp
#   src/fsm_states/StateStationKeeping.cpp
#   src/fsm_states/StateTrajectoryTracking.cpp
#   src/inner_wbc/PlanarImpedancePD.cpp
#   src/inner_wbc/PadForceDOB.cpp
#   src/inner_wbc/RhombicKernQP.cpp
#   src/outer_mppi_cuda/MppiRolloutThread.cpp
#   src/outer_mppi_cuda/mppi_rollout_kernel.cu
# )
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${EIGEN3_INCLUDE_DIR}
# )
# target_link_libraries(${PROJECT_NAME}
#   adhere_controller_common::adhere_controller_common
#   OsqpEigen::OsqpEigen
#   CUDA::cudart
# )
# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   rclcpp_lifecycle
#   hardware_interface
#   controller_interface
#   pluginlib
#   realtime_tools
#   adhere_control_msgs
# )
# pluginlib_export_plugin_description_file(controller_interface adhere_kamr_controller.xml)
#
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES adhere_kamr_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
