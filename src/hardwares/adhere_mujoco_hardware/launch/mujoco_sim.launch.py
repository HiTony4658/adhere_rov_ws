"""500Hz AdhereMujocoHardware passive-drift simulation with u=0, mirrored in MuJoCo+RViz2.

One-line usage:
    ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py
    ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py wave_profile:=station_keeping_gust
    ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py headless:=true use_rviz:=false
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory("simplified_v1_description")
    hardware_share = get_package_share_directory("adhere_mujoco_hardware")

    wave_profile = LaunchConfiguration("wave_profile")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_wave_profile = DeclareLaunchArgument(
        "wave_profile", default_value="calm",
        description="Ambient flow/wave environment: 'calm' or 'station_keeping_gust'.")
    declare_headless = DeclareLaunchArgument(
        "headless", default_value="false",
        description="If true, AdhereMujocoHardware skips the embedded GLFW/OpenGL viewer window.")
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Whether to launch RViz2 alongside.")

    xacro_file = os.path.join(description_share, "urdf", "simplified_v1.urdf.xacro")

    def _make_robot_description(context):
        processed = xacro.process_file(
            xacro_file,
            mappings={
                "wave_profile": context.launch_configurations["wave_profile"],
                "headless": context.launch_configurations["headless"],
            },
        ).toxml()
        return [
            Node(
                package="adhere_mujoco_hardware",
                executable="adhere_mujoco_node",
                name="adhere_mujoco_node",
                output="screen",
                parameters=[{"robot_description": processed}],
            )
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
            declare_wave_profile,
            declare_headless,
            declare_use_rviz,
            adhere_mujoco_node,
            rviz,
        ]
    )
