#!/usr/bin/env python3

import os
import launch 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Specify the name of the package
    pkg_name = "scout_navigation"
    namespace = "scout_mini"

    config_file_dir = os.path.join(get_package_share_directory(pkg_name), "config")

    # Arguments and parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    map_name = LaunchConfiguration("map_name", default="workshop_big_cartographer.yaml")

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value=use_sim_time,
        description="Use simulation (Gazebo) clock if true",
    )

    declare_map_name_arg = DeclareLaunchArgument(
        "map_name", default_value=map_name, description="Specify map name"
    )

    # Add namespace to robot_localization parameter files
    namespaced_ekf_localization_params = ReplaceString(
        source_file=os.path.join(config_file_dir, "ekf_localization_with_gps.yaml"),
        replacements={"namespace": namespace},
    )

    namespaced_nav2_params = ReplaceString(
        source_file=os.path.join(config_file_dir, "nav2_params.yaml"),
        replacements={"/namespace": ("/", namespace)}, #TODO: set if you use namespace or not. 
    )

    namespaced_nav2_params = RewrittenYaml(
        source_file=namespaced_nav2_params,
        root_key=namespace,
        param_rewrites={
            "yaml_filename": PathJoinSubstitution(
                [get_package_share_directory(pkg_name), "maps", map_name]
            ),
            "use_sim_time": use_sim_time
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
            os.path.join(
                get_package_share_directory(pkg_name),
                "rviz",
                "scout_mini_navigation.rviz",
            ),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        remappings=remapping,
    )

    # robot_localization_local_node = Node(
    #     namespace=namespace,
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_local_filter_node",
    #     output="screen",
    #     parameters=[namespaced_ekf_localization_params, {"use_sim_time": use_sim_time}],
    #     remappings=remapping
    #     + [
    #         ("odometry/filtered", "odometry/filtered/local"),
    #     ],
    # )

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
    ld.add_action(declare_map_name_arg)

    # Add the commands to the launch description
    # ld.add_action(robot_localization_local_node)
    # ld.add_action(robot_localization_global_node)
    # ld.add_action(navsat_transform_node)  # TODO: questa linea e' commentata per evitare di attivare tutte le volte il servizio GPS.
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
