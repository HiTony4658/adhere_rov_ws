"""500Hz AdhereMujocoHardware passive-drift simulation, mirrored in MuJoCo+RViz2.

Always starts in Calm Water; the wave-maker is switched on/off at runtime via the
`wave_console_node` interactive terminal (press 1/2), not via a launch-time argument.

One-line usage:
    ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py
    ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py headless:=true use_rviz:=false
    ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py spawn_wave_console:=false
"""
import os
import shutil

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory("simplified_v1_description")
    hardware_share = get_package_share_directory("adhere_mujoco_hardware")

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    spawn_wave_console = LaunchConfiguration("spawn_wave_console")

    declare_headless = DeclareLaunchArgument(
        "headless", default_value="false",
        description="If true, AdhereMujocoHardware skips the embedded GLFW/OpenGL viewer window.")
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Whether to launch RViz2 alongside.")
    declare_spawn_wave_console = DeclareLaunchArgument(
        "spawn_wave_console", default_value="true",
        description="If true, auto-spawn wave_console_node in a dedicated terminal window "
                     "(gnome-terminal or xterm). If neither is available, run "
                     "'ros2 run adhere_mujoco_hardware wave_console_node' manually instead.")

    xacro_file = os.path.join(description_share, "urdf", "simplified_v1.urdf.xacro")

    def _make_robot_description(context):
        # Pure-xacro decoupling (CLAUDE.md Phase 2.6 Stage 2.1): simplified_v1.urdf.xacro takes
        # no `headless` argument -- it is a 100% simulation-agnostic robot description.
        # `headless` is instead passed as a plain ROS 2 node parameter to adhere_mujoco_node,
        # which forwards it to AdhereMujocoHardware::setHeadless() after configure() but before
        # start_components() triggers start() (where MujocoViewer is actually constructed).
        processed = xacro.process_file(xacro_file).toxml()
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
                parameters=[{"robot_description": processed, "headless": headless_value}],
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

    def _make_wave_console(context):
        if context.launch_configurations.get("spawn_wave_console") != "true":
            return []
        inner_cmd = "ros2 run adhere_mujoco_hardware wave_console_node; exec bash"
        if shutil.which("gnome-terminal"):
            cmd = ["gnome-terminal", "--", "bash", "-c", inner_cmd]
        elif shutil.which("xterm"):
            cmd = ["xterm", "-e", "bash", "-c", inner_cmd]
        else:
            return []
        return [ExecuteProcess(cmd=cmd, output="screen")]

    wave_console = OpaqueFunction(function=_make_wave_console)

    return LaunchDescription(
        [
            declare_headless,
            declare_use_rviz,
            declare_spawn_wave_console,
            adhere_mujoco_node,
            rviz,
            wave_console,
        ]
    )
