"""Standalone Upper-Computer (Ground Control Station) teleop launch (CLAUDE.md Phase 3.2 Stage
2.5): joy_node (official ROS 2 joy package) + xbox_teleop_node + fsm_console_node. Zero
dependency on adhere_mujoco_hardware/adhere_controller/MuJoCo/CUDA/ros2_control -- runs
standalone on a lightweight remote laptop, controlling either the Dell Precision 7920 simulation
server or the Jetson AGX Xavier over ROS 2 DDS.

One-line usage:
    ros2 launch adhere_keyboard_input teleop_gcs.launch.py
    ros2 launch adhere_keyboard_input teleop_gcs.launch.py dev:=/dev/input/js1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    keyboard_input_share = get_package_share_directory("adhere_keyboard_input")
    xbox_teleop_yaml = os.path.join(keyboard_input_share, "config", "xbox_teleop.yaml")

    declare_dev = DeclareLaunchArgument(
        "dev", default_value="/dev/input/js0",
        description="Joystick device file passed to joy_node.")
    declare_deadzone = DeclareLaunchArgument(
        "joy_deadzone", default_value="0.05",
        description="joy_node's own raw-axis deadzone (xbox_teleop_node applies its own, "
                     "larger, configurable deadzone on top -- see config/xbox_teleop.yaml).")
    declare_autorepeat_rate = DeclareLaunchArgument(
        "autorepeat_rate", default_value="50.0",
        description="joy_node publish rate (Hz) on /joy, which xbox_teleop_node mirrors "
                     "1:1 onto /rov/teleop_cmd.")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "dev": LaunchConfiguration("dev"),
            "deadzone": LaunchConfiguration("joy_deadzone"),
            "autorepeat_rate": LaunchConfiguration("autorepeat_rate"),
        }],
    )

    xbox_teleop_node = Node(
        package="adhere_keyboard_input",
        executable="xbox_teleop_node",
        name="xbox_teleop_node",
        output="screen",
        parameters=[xbox_teleop_yaml],
    )

    # gnome-terminal (fallback xterm) running fsm_console_node in its own window (same idiom as
    # adhere_controller/launch/mujoco.launch.py's fsm_console block): a plain launch_ros
    # Node action does not reliably forward this launch process's stdin for interactive
    # std::cin-based terminals, so fsm_console_node needs its own real terminal.
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
        output="screen",
    )

    return LaunchDescription(
        [
            declare_dev,
            declare_deadzone,
            declare_autorepeat_rate,
            joy_node,
            xbox_teleop_node,
            fsm_console,
        ]
    )
