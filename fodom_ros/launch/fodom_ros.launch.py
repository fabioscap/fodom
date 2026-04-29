from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("odom_ros")
    config = os.path.join(pkg_dir, "config", "odom_ros.yaml")
    rviz_config = os.path.join(pkg_dir, "config", "odom_rviz.rviz")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="false", description="Launch RViz2"
    )

    config_arg = DeclareLaunchArgument(
        "config", default_value=config, description="Path to the odom config YAML"
    )

    odom_node = Node(
        package="odom_ros",
        executable="odom_ros_node",
        name="odom_ros_node",
        parameters=[LaunchConfiguration("config")],
        remappings=[("points", "/ouster/points")],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription([use_rviz_arg, config_arg, odom_node, rviz_node])
