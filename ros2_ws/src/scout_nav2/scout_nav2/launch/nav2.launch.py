import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    slam_arg = DeclareLaunchArgument(
        name="slam",
        default_value="False",
        description="Launch SLAM or launch localization and navigation",
        choices=["True", "False", "true", "false"],
    )

    localization_arg = DeclareLaunchArgument(
        name="localization",
        default_value="slam_toolbox",
        description="Launch localization with AMCL algorithm or SLAM toolbox",
        choices=["amcl", "slam_toolbox"],
    )

    simulation_arg = DeclareLaunchArgument(
        name="simulation",
        default_value="true",
        description="Launch simulation with gazebo or launch real robot navigation",
        choices=["True", "False", "true", "false"],
    )

    return LaunchDescription(
        [
            slam_arg,
            simulation_arg,
            localization_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):
    nav2_bringup_dir = get_package_share_directory("nav2_bringup_custom")
    nav2_launch_file = os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
    scout_nav2_dir = get_package_share_directory("scout_nav2")

    slam = LaunchConfiguration("slam").perform(context).lower()
    simulation = LaunchConfiguration("simulation").perform(context).lower()

    if simulation == "true":
        map_file = "warehouse/map_slam_v2.yaml"
        use_sim_time = "true"

        if slam == "true":
            params_file_name = "sim_lidar3d_amcl.yaml"
        else:
            if (LaunchConfiguration("localization").perform(context) == "slam_toolbox"):
                params_file_name = "sim_slam_localization.yaml"
            else:
                params_file_name = "sim_lidar3d_amcl.yaml"
    else:
        map_file = "/home/region/nav2_ws/src/scout_nav2/scout_nav2/maps/new_map/map_lidar3d_v3.yaml"
        use_sim_time = "false"

        if slam == "true":
            params_file_name = "scout_amcl.yaml"
        else:
            if (LaunchConfiguration("localization").perform(context) == "slam_toolbox"):
                params_file_name = "scout_slam_localization.yaml"
            else:
                params_file_name = "scout_amcl.yaml"

    nav2_params_file = os.path.join(scout_nav2_dir, "params", params_file_name)
    map_yaml_file = os.path.join(scout_nav2_dir, "maps", map_file)

    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "slam": "True",
            "map": "",
        }.items(),
        condition=IfCondition(PythonExpression(["'" , LaunchConfiguration("slam"), "' == 'true'"]))
    )

    nav2_localization_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "slam": "False",
            "map": map_yaml_file,
            "localization": LaunchConfiguration("localization")
        }.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("slam") , "' == 'false'"]))
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
            "--frame-id", "world", "--child-frame-id", "map",
        ],
    )

    rviz_default_config_file = os.path.join(scout_nav2_dir, "rviz", "nav2.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_default_config_file],
    )

    return [nav2_slam_launch, nav2_localization_navigation_launch, rviz_node, static_tf]
