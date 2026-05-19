from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    pcd_file = PathJoinSubstitution([
        FindPackageShare("mcl3d_ros"),
        "data",
        "map_mcl_3dl.pcd",
    ])
    yaml_file = PathJoinSubstitution([
        FindPackageShare("mcl3d_ros"),
        "data",
        "dist_map_mcl_3dl.yaml",
    ])

    args = [
        DeclareLaunchArgument("node_name", default_value="pc_to_df"),
        DeclareLaunchArgument("from_pcd_file", default_value="true"),
        DeclareLaunchArgument("pcd_file", default_value=pcd_file),
        DeclareLaunchArgument("map_points_name", default_value="/map_points"),
        DeclareLaunchArgument("map_file_name", default_value="dist_map_mcl_3dl.bin"),
        DeclareLaunchArgument("resolution", default_value="5.0"),
        DeclareLaunchArgument("sub_map_resolution", default_value="0.1"),
        DeclareLaunchArgument("map_margin", default_value="1.0"),
        DeclareLaunchArgument("yaml_file_path", default_value=yaml_file),
    ]

    from_pcd = Node(
        name="pc_to_df",
        package="mcl3d_ros",
        executable="pc_to_df",
        output="screen",
        condition=IfCondition(LaunchConfiguration("from_pcd_file")),
        arguments=[
            LaunchConfiguration("pcd_file"),
            LaunchConfiguration("map_file_name"),
            LaunchConfiguration("resolution"),
            LaunchConfiguration("sub_map_resolution"),
            LaunchConfiguration("map_margin"),
            LaunchConfiguration("yaml_file_path"),
        ],
    )

    from_topic = Node(
        name=LaunchConfiguration("node_name"),
        package="mcl3d_ros",
        executable="pc_to_df",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("from_pcd_file")),
        parameters=[{
            "map_points_name": LaunchConfiguration("map_points_name"),
            "map_file_name": LaunchConfiguration("map_file_name"),
            "resolution": typed("resolution", float),
            "sub_map_resolution": typed("sub_map_resolution", float),
            "map_margin": typed("map_margin", float),
            "yaml_file_path": LaunchConfiguration("yaml_file_path"),
        }],
    )

    return LaunchDescription(args + [from_pcd, from_topic])
