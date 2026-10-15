"""Static geometry inspection view: robot_state_publisher + RViz2, no physics.

One-line usage:
    ros2 launch simplified_v1_description visualize.launch.py
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("simplified_v1_description")
    xacro_file = os.path.join(pkg_share, "urdf", "simplified_v1.urdf.xacro")
    rviz_config = os.path.join(pkg_share, "config", "visualize.rviz")

    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Every joint in this model is `type="fixed"` (robot.xacro roots the whole tree at a
    # "world" link; the only real DOF -- the floating base -- is owned by
    # AdhereMujocoHardware, not by any URDF joint). robot_state_publisher broadcasts
    # tf_static for all fixed joints directly from the URDF, so no joint_state_publisher
    # (GUI or otherwise) is needed for this static geometry view.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            rviz,
        ]
    )
