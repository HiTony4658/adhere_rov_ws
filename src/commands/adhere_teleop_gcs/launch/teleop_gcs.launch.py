"""Standalone Upper-Computer (Ground Control Station) teleop launch (CLAUDE.md Phase 3.2 Stage
2.5): joy_node (official ROS 2 joy package) + xbox_teleop_node + fsm_console_node. Zero
dependency on adhere_mujoco_hardware/adhere_controller/MuJoCo/CUDA/ros2_control -- runs
standalone on a lightweight remote laptop, controlling either the Dell Precision 7920 simulation
server or the Jetson AGX Xavier over ROS 2 DDS.

One-line usage:
    ros2 launch adhere_teleop_gcs teleop_gcs.launch.py
    ros2 launch adhere_teleop_gcs teleop_gcs.launch.py dev:=/dev/input/js1
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleop_gcs_pkg_share = get_package_share_directory("adhere_teleop_gcs")
    xbox_teleop_yaml = os.path.join(teleop_gcs_pkg_share, "config", "xbox_teleop.yaml")

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
        package="adhere_teleop_gcs",
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
            "gnome-terminal --wait --title='ROV FSM Console' -- "
            "ros2 run adhere_teleop_gcs fsm_console_node; "
            "elif command -v xterm >/dev/null 2>&1; then "
            "xterm -T 'ROV FSM Console' -e "
            "ros2 run adhere_teleop_gcs fsm_console_node; "
            "else echo 'fsm_console: neither gnome-terminal nor xterm found, run "
            "\"ros2 run adhere_teleop_gcs fsm_console_node\" manually' >&2; fi",
        ],
        output="screen",
    )

    # Root-cause fix for the Ctrl+C orphan-process bug: `gnome-terminal --wait` does not run
    # fsm_console_node as a direct child of this launch's process tree -- it D-Bus-activates the
    # shared `gnome-terminal-server` daemon, which forks the actual `ros2 run
    # adhere_teleop_gcs fsm_console_node` under that daemon instead. `ros2 launch`'s own
    # SIGINT/Shutdown propagation only reaches processes in its own tree (confirmed empirically:
    # joy_node/xbox_teleop_node/the fsm_console ExecuteProcess wrapper all exit cleanly on
    # SIGINT, but the gnome-terminal-server-spawned fsm_console_node process survives, reparented
    # to systemd --user). `pkill -f` matches the full command line, so it reaches that detached
    # tree regardless of its real parent. This MUST be a synchronous `OpaqueFunction` (blocking
    # `subprocess.call`), not an `ExecuteProcess` action: empirically verified that `ros2
    # launch`'s Ctrl-C path calls `LaunchService._shutdown(force_sync=True)`, which emits the
    # `Shutdown` event synchronously and tears the asyncio loop down immediately after -- an
    # `ExecuteProcess` queued from `on_shutdown` never actually gets scheduled/started in that
    # window (confirmed with a minimal repro: a `LogInfo` in the same `on_shutdown` list fires,
    # but a sibling `ExecuteProcess` never produces output), while a blocking call inside
    # `OpaqueFunction` runs to completion before shutdown proceeds. Phase 5.0 deleted
    # fsm_console_node's nested state_cmd_console_node child (and that node entirely) along with
    # the per-state setpoint-terminal auto-spawn feature, so only fsm_console_node itself needs
    # this treatment now -- re-evaluated and kept because fsm_console_node is still
    # gnome-terminal-spawned by this same launch file below.
    def _kill_orphaned_consoles(context):
        subprocess.call(["bash", "-c", "pkill -TERM -f fsm_console_node; true"])
        return None

    kill_orphaned_consoles_on_shutdown = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=_kill_orphaned_consoles)])
    )

    return LaunchDescription(
        [
            declare_dev,
            declare_deadzone,
            declare_autorepeat_rate,
            joy_node,
            xbox_teleop_node,
            fsm_console,
            kill_orphaned_consoles_on_shutdown,
        ]
    )
