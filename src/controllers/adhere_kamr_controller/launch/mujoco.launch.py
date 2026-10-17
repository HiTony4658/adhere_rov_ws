"""Closed-loop AdhereKamrController simulation: 500Hz AdhereMujocoHardware + FSM, mirrored in
MuJoCo+RViz2 (CLAUDE.md Phase 3.1.1 Stage 2.2).

Boots adhere_mujoco_node with autostart_controllers:=['adhere_kamr_controller'] and
simplified_v1_description/config/robot_control.yaml, which load/configure/activate
AdhereKamrController in-process (via the generic autostart_controllers parameter rather than a
separate controller_manager/spawner process, since this avoids a second process having to
discover the controller_manager node's services over DDS -- a discovery path that can be
unreliable inside sandboxed/CI shells). The FSM always boots into its default PASSIVE state
(u=0); a uniform 250Hz inner loop / 25Hz outer loop then runs inside AdhereKamrController::
update() once activated.

One-line usage:
    ros2 launch adhere_kamr_controller mujoco.launch.py
    ros2 launch adhere_kamr_controller mujoco.launch.py headless:=true spawn_fsm_console:=false

launch_teleop (default true, CLAUDE.md Phase 3.2 Stage 2.5) includes
adhere_keyboard_input/launch/teleop_gcs.launch.py (joy_node + xbox_teleop_node +
fsm_console_node) for single-workstation convenience. Set launch_teleop:=false when this
simulation server is controlled remotely from a separate Upper-Computer (GCS) laptop running
teleop_gcs.launch.py on its own:
    ros2 launch adhere_kamr_controller mujoco.launch.py launch_teleop:=false

Zero-recompile model switching (CLAUDE.md Phase 2.12 Section 1): pkg_description/mjcf_file/
xacro_file default to simplified_v1_description's rov_wall.xml + its xacro entry point, exactly
as in adhere_mujoco_hardware/launch/mujoco_sim.launch.py.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        description="Robot description package providing the xacro/MJCF model + "
                     "robot_control.yaml (CLAUDE.md Phase 2.12).")
    declare_mjcf_file = DeclareLaunchArgument(
        "mjcf_file", default_value="mjcf/rov_wall.xml",
        description="MJCF file path, relative to pkg_description's share directory.")
    declare_xacro_file = DeclareLaunchArgument(
        "xacro_file", default_value="urdf/simplified_v1.urdf.xacro",
        description="URDF/xacro entry-point file path, relative to pkg_description's share "
                     "directory.")
    declare_robot_control_yaml = DeclareLaunchArgument(
        "robot_control_yaml", default_value="config/robot_control.yaml",
        description="controller_manager + adhere_kamr_controller parameter file path, relative "
                     "to pkg_description's share directory.")
    declare_launch_teleop = DeclareLaunchArgument(
        "launch_teleop", default_value="true",
        description="If true, includes adhere_keyboard_input/launch/teleop_gcs.launch.py "
                     "(joy_node + xbox_teleop_node + fsm_console_node) for single-workstation "
                     "convenience. Set false when a separate Upper-Computer (GCS) laptop "
                     "already runs teleop_gcs.launch.py against this server (CLAUDE.md Phase "
                     "3.2 Stage 2.5).")

    def _make_robot_description(context):
        # Mirrors adhere_mujoco_hardware/launch/mujoco_sim.launch.py's pure-xacro decoupling
        # (CLAUDE.md Phase 2.6 Stage 2.1): `headless` stays a plain node parameter, never a
        # xacro argument.
        pkg_description_value = context.launch_configurations["pkg_description"]
        mjcf_file_value = context.launch_configurations["mjcf_file"]
        xacro_file_value = context.launch_configurations["xacro_file"]
        robot_control_yaml_value = context.launch_configurations["robot_control_yaml"]
        description_share = get_package_share_directory(pkg_description_value)
        xacro_file = os.path.join(description_share, xacro_file_value)
        processed = xacro.process_file(xacro_file).toxml()
        robot_control_yaml = os.path.join(description_share, robot_control_yaml_value)
        headless_value = context.launch_configurations["headless"] == "true"
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
                parameters=[
                    robot_control_yaml,
                    {
                        "robot_description": processed,
                        "headless": headless_value,
                        "pkg_description": pkg_description_value,
                        "mjcf_file": mjcf_file_value,
                        "autostart_controllers": ["adhere_kamr_controller"],
                    },
                ],
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

    teleop_gcs_share = get_package_share_directory("adhere_keyboard_input")
    teleop_gcs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_gcs_share, "launch", "teleop_gcs.launch.py")),
        condition=IfCondition(LaunchConfiguration("launch_teleop")),
    )

    return LaunchDescription(
        [
            declare_headless,
            declare_use_rviz,
            declare_pkg_description,
            declare_mjcf_file,
            declare_xacro_file,
            declare_robot_control_yaml,
            declare_launch_teleop,
            adhere_mujoco_node,
            rviz,
            teleop_gcs,
        ]
    )
