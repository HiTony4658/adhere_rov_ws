"""Model-centric entry point for the full ROV simulation stack (CLAUDE.md Phase 3.4): owned by
`simplified_v1_description` so that switching to a new ROV model only requires that model's own
description package to provide this same launch file -- `adhere_mujoco_hardware` and
`adhere_controller` stay pure, generic toolboxes with zero launch files of their own.

Supersedes both of the former launch files it replaces:
  - adhere_mujoco_hardware/launch/mujoco_sim.launch.py (open-loop, 0 controllers)
  - adhere_controller/launch/mujoco.launch.py (closed-loop, AdhereController autostarted)
Both behaviors are reachable from this single file via `autostart_controllers`/`launch_teleop`.

One-line usage:
    ros2 launch simplified_v1_description mujoco.launch.py
    ros2 launch simplified_v1_description mujoco.launch.py headless:=true launch_teleop:=false
    ros2 launch simplified_v1_description mujoco.launch.py autostart_controllers:="[]" \
        launch_teleop:=false

Zero-recompile model switching (CLAUDE.md Sec.3.1): pkg_description/mjcf_file/xacro_file/
robot_control_yaml default to this package's own files; overriding them (plus running this same
launch file from a different description package) switches ROV models with zero C++
recompilation.
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_headless = DeclareLaunchArgument(
        "headless", default_value="false",
        description="If true, AdhereMujocoHardware skips the embedded GLFW/OpenGL viewer window.")
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Whether to launch RViz2 alongside.")
    declare_pkg_description = DeclareLaunchArgument(
        "pkg_description", default_value="simplified_v1_description",
        description="Robot description package providing the xacro/MJCF model + "
                     "robot_control.yaml (CLAUDE.md Sec.3.1): override to switch ROV models "
                     "with zero C++ recompilation.")
    declare_mjcf_file = DeclareLaunchArgument(
        "mjcf_file", default_value="mjcf/rov_wall.xml",
        description="MJCF file path, relative to pkg_description's share directory.")
    declare_xacro_file = DeclareLaunchArgument(
        "xacro_file", default_value="urdf/simplified_v1.urdf.xacro",
        description="URDF/xacro entry-point file path, relative to pkg_description's share "
                     "directory.")
    declare_robot_control_yaml = DeclareLaunchArgument(
        "robot_control_yaml", default_value="config/robot_control.yaml",
        description="controller_manager + adhere_controller parameter file path, relative to "
                     "pkg_description's share directory.")
    declare_autostart_controllers = DeclareLaunchArgument(
        "autostart_controllers", default_value="['adhere_controller']",
        description="Generic, controller-agnostic list of controller names to load/configure/"
                     "activate in-process (CLAUDE.md Phase 3.1.1 Stage 2.2). Pass '[]' for the "
                     "former open-loop-only behavior (u=0, no FSM).")
    declare_estimation_mode = DeclareLaunchArgument(
        "estimation_mode", default_value="sensor_fusion",
        description="Phase 3.5 dual state-acquisition mode for AdhereController's "
                     "CtrlInterfaces::active_state: 'ground_truth' (noise-free hardware "
                     "odometry) or 'sensor_fusion' (RovStateEstimator fusing the noisy "
                     "multi-rate IMU/Depth/DVL/load-cell sensors; ground truth is then only "
                     "used to fill FsmState::ground_truth_pose for offline evaluation).")
    declare_launch_teleop = DeclareLaunchArgument(
        "launch_teleop", default_value="true",
        description="If true, includes adhere_keyboard_input/launch/teleop_gcs.launch.py "
                     "(joy_node + xbox_teleop_node + fsm_console_node) for single-workstation "
                     "convenience. Set false when a separate Upper-Computer (GCS) laptop "
                     "already runs teleop_gcs.launch.py against this server (CLAUDE.md Phase "
                     "3.2 Stage 2.5), or when autostart_controllers is empty.")

    def _make_robot_description(context):
        # Pure-xacro decoupling (CLAUDE.md Sec.3.1): `headless` stays a plain adhere_mujoco_node
        # parameter, never a xacro argument -- simplified_v1.urdf.xacro is 100%
        # simulation-agnostic.
        pkg_description_value = context.launch_configurations["pkg_description"]
        mjcf_file_value = context.launch_configurations["mjcf_file"]
        xacro_file_value = context.launch_configurations["xacro_file"]
        robot_control_yaml_value = context.launch_configurations["robot_control_yaml"]
        pkg_description_share = get_package_share_directory(pkg_description_value)
        xacro_file = os.path.join(pkg_description_share, xacro_file_value)
        processed = xacro.process_file(xacro_file).toxml()
        robot_control_yaml = os.path.join(pkg_description_share, robot_control_yaml_value)
        headless_value = context.launch_configurations["headless"] == "true"
        # yaml.safe_load (not a bare str) so a non-default override (e.g.
        # autostart_controllers:="[]") is passed to adhere_mujoco_node as an actual string
        # array, not a single literal string (CLAUDE.md Phase 3.1.1 Stage 2.2).
        autostart_controllers_value = yaml.safe_load(
            context.launch_configurations["autostart_controllers"])
        adhere_mujoco_node_parameters = {
            "robot_description": processed,
            "headless": headless_value,
            "pkg_description": pkg_description_value,
            "mjcf_file": mjcf_file_value,
            "estimation_mode": context.launch_configurations["estimation_mode"],
        }
        # launch_ros cannot evaluate an empty-list ROS parameter (it can't infer the array's
        # element type with zero elements), so the key is simply omitted when empty -- the
        # C++-side declare_parameter default ({}) already matches this launch argument's own
        # default, so the parameter's effective value is unaffected either way.
        if autostart_controllers_value:
            adhere_mujoco_node_parameters["autostart_controllers"] = autostart_controllers_value

        # RViz config now ships from pkg_description's own config/ (CLAUDE.md Phase 3.7: moved
        # out of adhere_mujoco_hardware, which owns zero model/visualization config files),
        # resolved here (not at module scope) since pkg_description_share already needs the
        # launch-context substitution above.
        rviz_config = os.path.join(pkg_description_share, "config", "mujoco_sim.rviz")
        rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )

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
                parameters=[robot_control_yaml, adhere_mujoco_node_parameters],
            ),
            rviz,
        ]

    adhere_mujoco_node = OpaqueFunction(function=_make_robot_description)

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
            declare_autostart_controllers,
            declare_estimation_mode,
            declare_launch_teleop,
            adhere_mujoco_node,
            teleop_gcs,
        ]
    )
