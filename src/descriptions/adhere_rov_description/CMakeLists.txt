cmake_minimum_required(VERSION 3.5)
project(adhere_rov_description)

find_package(ament_cmake REQUIRED)

# TODO(Phase 2): install actual urdf/xacro/mjcf/meshes/config/launch content
# once rov_wall.xml (MJCF) and adhere_rov.urdf.xacro are authored.
install(
  DIRECTORY urdf xacro mjcf meshes config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
