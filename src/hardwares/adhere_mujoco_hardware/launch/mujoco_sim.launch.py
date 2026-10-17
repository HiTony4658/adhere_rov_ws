"""500Hz AdhereMujocoHardware passive-drift simulation, mirrored in MuJoCo+RViz2.

Always starts in Calm Water; the wave-maker is switched on/off at runtime from the embedded
MuJoCo viewer itself (press 1/2, or use the left panel's "Wave Environment" radio buttons),
not via a launch-time argument.

One-line usage:
    ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py
    ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py headless:=true use_rviz:=false

Pure open-loop hardware launch (CLAUDE.md Phase 3.1.1 Stage 2.2): this package has zero
knowledge of any specific controller or teleop console. autostart_controllers defaults to an
empty list, so adhere_mujoco_node always boots with exactly 0 active controllers and
hw_commands_ stays at u=0. Closed-loop control has its own, separate launch file in the
controller package that wants it.

Zero-recompile model switching (CLAUDE.md Phase 2.12 Section 1): pkg_description/mjcf_file/
xacro_file default to simplified_v1_description's rov_wall.xml + its xacro entry point.
Switching to a future robot description package requires only overriding these three launch
arguments -- no AdhereMujocoHardware C++ recompilation:
    ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py \
        pkg_description:=my_new_rov_description mjcf_file:=mjcf/my_rov.xml \
        xacro_file:=urdf/my_rov.urdf.xacro
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_share = get_package_share_directory("adhere_mujoco_hardware")

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_headless = DeclareLaunchArgument(
        "headless", default_value="false",
        description="If true, AdhereMujocoHardware skips the embedded GLFW/OpenGL viewer window.")
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Whether to launch RViz2 alongside.")
    declare_pkg_description = DeclareLaunchArgument(
        "pkg_description", default_value="simplified_v1_description",
        description="Robot description package providing the xacro/MJCF model (CLAUDE.md "
                     "Phase 2.12): override to switch ROV models with zero C++ recompilation.")
    declare_mjcf_file = DeclareLaunchArgument(
        "mjcf_file", default_value="mjcf/rov_wall.xml",
        description="MJCF file path, relative to pkg_description's share directory.")
    declare_xacro_file = DeclareLaunchArgument(
        "xacro_file", default_value="urdf/simplified_v1.urdf.xacro",
        description="URDF/xacro entry-point file path, relative to pkg_description's share "
                     "directory.")
    declare_autostart_controllers = DeclareLaunchArgument(
        "autostart_controllers", default_value="[]",
        description="Generic, controller-agnostic list of controller names to load/configure/"
                     "activate in-process (CLAUDE.md Phase 3.1.1 Stage 2.2). Empty by default, "
                     "so this launch file always starts exactly 0 controllers (u=0).")

    def _make_robot_description(context):
        # Pure-xacro decoupling (CLAUDE.md Phase 2.6 Stage 2.1): simplified_v1.urdf.xacro takes
        # no `headless` argument -- it is a 100% simulation-agnostic robot description.
        # `headless` is instead passed as a plain ROS 2 node parameter to adhere_mujoco_node,
        # which forwards it to AdhereMujocoHardware::setHeadless() after configure() but before
        # start_components() triggers start() (where MujocoViewer is actually constructed).
        pkg_description_value = context.launch_configurations["pkg_description"]
        mjcf_file_value = context.launch_configurations["mjcf_file"]
        xacro_file_value = context.launch_configurations["xacro_file"]
        description_share = get_package_share_directory(pkg_description_value)
        xacro_file = os.path.join(description_share, xacro_file_value)
        processed = xacro.process_file(xacro_file).toxml()
        headless_value = context.launch_configurations["headless"] == "true"
        # yaml.safe_load (not a bare str) so a non-empty override (e.g.
        # autostart_controllers:="['my_controller']") is passed to adhere_mujoco_node as an
        # actual string array, not a single literal string (CLAUDE.md Phase 3.1.1 Stage 2.2).
        autostart_controllers_value = yaml.safe_load(
            context.launch_configurations["autostart_controllers"])
        adhere_mujoco_node_parameters = {
            "robot_description": processed,
            "headless": headless_value,
            "pkg_description": pkg_description_value,
            "mjcf_file": mjcf_file_value,
        }
        # launch_ros cannot evaluate an empty-list ROS parameter (it can't infer the array's
        # element type with zero elements), so the key is simply omitted when empty -- the
        # C++-side declare_parameter default ({}) already matches this launch argument's own
        # default, so the parameter's effective value is unaffected either way.
        if autostart_controllers_value:
            adhere_mujoco_node_parameters["autostart_controllers"] = autostart_controllers_value
        return [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": processed}],
            ),
            Node(
                package="adhere_mujoco_hardware",
                executable="adhere_mujoco_node",
                name="adhere_mujoco_node",
                output="screen",
                parameters=[adhere_mujoco_node_parameters],
            ),
        ]

    adhere_mujoco_node = OpaqueFunction(function=_make_robot_description)

    rviz_config = os.path.join(hardware_share, "config", "mujoco_sim.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            declare_headless,
            declare_use_rviz,
            declare_pkg_description,
            declare_mjcf_file,
            declare_xacro_file,
            declare_autostart_controllers,
            adhere_mujoco_node,
            rviz,
        ]
    )
