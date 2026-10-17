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

spawn_fsm_console (default true) opens the interactive FSM terminal console
(adhere_keyboard_input's fsm_console_node) in its own gnome-terminal/xterm window; set
spawn_fsm_console:=false and run it manually instead:
    ros2 run adhere_keyboard_input fsm_console_node

Zero-recompile model switching (CLAUDE.md Phase 2.12 Section 1): pkg_description/mjcf_file/
xacro_file default to simplified_v1_description's rov_wall.xml + its xacro entry point, exactly
as in adhere_mujoco_hardware/launch/mujoco_sim.launch.py.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
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
    declare_spawn_fsm_console = DeclareLaunchArgument(
        "spawn_fsm_console", default_value="true",
        description="If true, opens adhere_keyboard_input's fsm_console_node in a dedicated "
                     "gnome-terminal/xterm window.")

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

    # gnome-terminal (fallback xterm) running fsm_console_node in its own window, so it keeps
    # reading stdin independently of this launch file's own terminal/log output. Guarded by
    # `command -v` + if/elif so one missing terminal emulator falls back to the other instead of
    # failing the whole launch.
    fsm_console = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "if command -v gnome-terminal >/dev/null 2>&1; then "
            "gnome-terminal --title='ROV FSM Console' -- "
            "ros2 run adhere_keyboard_input fsm_console_node; "
            "elif command -v xterm >/dev/null 2>&1; then "
            "xterm -T 'ROV FSM Console' -e "
            "ros2 run adhere_keyboard_input fsm_console_node; "
            "else echo 'fsm_console: neither gnome-terminal nor xterm found, run "
            "\"ros2 run adhere_keyboard_input fsm_console_node\" manually' >&2; fi",
        ],
        condition=IfCondition(LaunchConfiguration("spawn_fsm_console")),
        output="screen",
    )

    return LaunchDescription(
        [
            declare_headless,
            declare_use_rviz,
            declare_pkg_description,
            declare_mjcf_file,
            declare_xacro_file,
            declare_robot_control_yaml,
            declare_spawn_fsm_console,
            adhere_mujoco_node,
            rviz,
            fsm_console,
        ]
    )
