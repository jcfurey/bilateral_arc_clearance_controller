"""Launch the BAC evaluation filter with configurable topic remappings."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("bilateral_arc_clearance_controller"), "config", "bac_filter.yaml"]
    )

    arguments = [
        DeclareLaunchArgument(
            "params_file", default_value=default_params, description="Filter parameter file"
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Run the filter tick and freshness checks on /clock",
        ),
        DeclareLaunchArgument("scan", default_value="/scan"),
        DeclareLaunchArgument("odom", default_value="/odom"),
        DeclareLaunchArgument("cmd_vel_in", default_value="/nav_cmd_vel"),
        DeclareLaunchArgument("cmd_vel_out", default_value="/cmd_vel"),
    ]
    node = Node(
        package="bilateral_arc_clearance_controller",
        executable="bac_filter_node",
        name="bac_filter",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ],
        remappings=[
            ("scan", LaunchConfiguration("scan")),
            ("odom", LaunchConfiguration("odom")),
            ("cmd_vel_in", LaunchConfiguration("cmd_vel_in")),
            ("cmd_vel_out", LaunchConfiguration("cmd_vel_out")),
        ],
    )
    return LaunchDescription(arguments + [node])
