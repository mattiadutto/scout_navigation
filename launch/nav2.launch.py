#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Specify the name of the package
    pkg_name = "scout_navigation"
    namespace = ""
    config_file_dir = os.path.join(get_package_share_directory(pkg_name), "config")

    # Arguments and parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    map_name = LaunchConfiguration("map_name", default="slam_farm.yaml")
    namespace = LaunchConfiguration("namespace", default="")
    ekf_params_file = LaunchConfiguration(
        "ekf_params_file", default="ekf_localization_with_gps.yaml"
    )
    nav2_params_file = LaunchConfiguration(
        "nav2_params_file", default="nav2_params.yaml"
    )
    rviz_params_file = LaunchConfiguration(
        "rviz_params_file", default="scout_mini_navigation.rviz"
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value=use_sim_time,
        description="Use simulation (Gazebo) clock if true",
    )

    declare_user_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value=use_rviz, description="Run rviz if true"
    )

    declare_map_name_arg = DeclareLaunchArgument(
        "map_name", default_value=map_name, description="Specify map name"
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=namespace,
        description="Specify namespace of the robot",
    )

    declare_nav2_params_file = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=nav2_params_file,
        description="Specify the nav 2 configuration file",
    )

    declare_rviz_params_file = DeclareLaunchArgument(
        "rviz_params_file",
        default_value=rviz_params_file,
        description="Specify the rviz 2 configuration file",
    )

    declare_ekf_params_file = DeclareLaunchArgument(
        "ekf_params_file",
        default_value=ekf_params_file,
        description="Specify the ekf configuration file",
    )

    # Add namespace to robot_localization parameter files
    namespaced_ekf_localization_params = ReplaceString(
        source_file=PathJoinSubstitution([config_file_dir, ekf_params_file]),
        replacements={"namespace": namespace},
    )

    namespaced_nav2_params = ReplaceString(
        source_file=PathJoinSubstitution([config_file_dir, nav2_params_file]),
        replacements={
            "/namespace": ("/", namespace) if namespace != "" else ""
        },  # TODO: set if you use namespace or not.
    )

    namespaced_nav2_params = RewrittenYaml(
        source_file=namespaced_nav2_params,
        root_key=namespace,
        param_rewrites={
            "yaml_filename": PathJoinSubstitution(
                [get_package_share_directory(pkg_name), "maps", map_name]
            ),
            "use_sim_time": use_sim_time,
        },
        convert_types=True,
    )

    remapping = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    # Nodes
    rviz2_node = Node(
        namespace=namespace,
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [get_package_share_directory(pkg_name), "rviz", rviz_params_file]
            ),
        ],
        condition=IfCondition(use_rviz),
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        remappings=remapping,
    )

    robot_localization_local_node = Node(
        namespace=namespace,
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_filter_node",
        output="screen",
        parameters=[namespaced_ekf_localization_params, {"use_sim_time": use_sim_time}],
        remappings=remapping
        + [
            ("odometry/filtered", "odometry/filtered/local"),
        ],
    )

    # robot_localization_global_node = Node(
    #     namespace=namespace,
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_global_filter_node",
    #     output="screen",
    #     parameters=[namespaced_ekf_localization_params, {"use_sim_time": use_sim_time}],
    #     remappings=remapping
    #     + [
    #         ("odometry/filtered", "odometry/filtered/global"),
    #     ],
    # )

    # navsat_transform_node = Node(
    #     namespace=namespace,
    #     package="robot_localization",
    #     executable="navsat_transform_node",
    #     name="navsat_transform_node",
    #     output="screen",
    #     parameters=[namespaced_ekf_localization_params, {"use_sim_time": use_sim_time}],
    #     remappings=remapping
    #     + [
    #         ("imu/data", "imu"),
    #         ("gps/fix", "gps"),
    #         ("odometry/filtered", "odometry/filtered/global"),
    #     ],
    # )

    nav2_bt_node = Node(
        namespace=namespace,
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        respawn=False,
        respawn_delay=2.0,
        parameters=[namespaced_nav2_params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=remapping,
    )

    nav2_planner_node = Node(
        namespace=namespace,
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        respawn=False,
        respawn_delay=2.0,
        parameters=[namespaced_nav2_params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=remapping,
    )

    nav2_controller_node = Node(
        namespace=namespace,
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        respawn=False,
        respawn_delay=2.0,
        parameters=[namespaced_nav2_params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=remapping
        + [
            ("/cmd_vel", "cmd_vel_nav"),
        ],
    )

    nav2_map_server_node = Node(
        namespace=namespace,
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        respawn=False,
        respawn_delay=2.0,
        parameters=[namespaced_nav2_params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=[("/tf", "tf")],
    )

    nav2_amcl_node = Node(
        namespace=namespace,
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        respawn=False,
        respawn_delay=2.0,
        parameters=[namespaced_nav2_params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=remapping,
    )

    nav2_behavior_server_node = Node(
        namespace=namespace,
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        respawn=False,
        respawn_delay=2.0,
        parameters=[namespaced_nav2_params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=remapping,
    )

    nav2_smoother_server_node = Node(
        namespace=namespace,
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        respawn=False,
        respawn_delay=2.0,
        parameters=[namespaced_nav2_params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=remapping,
    )

    nav2_waypoint_follower_node = Node(
        namespace=namespace,
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        respawn=False,
        respawn_delay=2.0,
        parameters=[namespaced_nav2_params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=remapping,
    )

    nav2_lifecycle_manager_node = Node(
        namespace=namespace,
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"bond_timeout": 4.0},
            {"attempt_respawn_reconnection": True},
            {"bond_respawn_max_duration": 10.0},
            {
                "node_names": [
                    "map_server",
                    "amcl",
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                ]
            },
        ],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_user_rviz_arg)
    ld.add_action(declare_map_name_arg)
    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_ekf_params_file)
    ld.add_action(declare_nav2_params_file)
    ld.add_action(declare_rviz_params_file)

    # Add the commands to the launch description
    ld.add_action(robot_localization_local_node)
    # ld.add_action(robot_localization_global_node)
    # ld.add_action(navsat_transform_node)  # TODO: this line is commented for avoid to startup each time the GPS in simulation.
    ld.add_action(nav2_map_server_node)
    ld.add_action(nav2_amcl_node)
    ld.add_action(nav2_controller_node)
    ld.add_action(nav2_smoother_server_node)
    ld.add_action(nav2_planner_node)
    ld.add_action(nav2_behavior_server_node)
    ld.add_action(nav2_bt_node)
    ld.add_action(nav2_waypoint_follower_node)
    ld.add_action(nav2_lifecycle_manager_node)

    ld.add_action(rviz2_node)

    return ld
