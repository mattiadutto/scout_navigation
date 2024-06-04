#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch.conditions import IfCondition

def generate_launch_description():
    # Specify the name of the package
    pkg_name = "scout_navigation"
    namespace = "scout_mini"

    config_file_dir = os.path.join(get_package_share_directory(pkg_name), "config")

    # Arguments and parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value=use_sim_time,
        description="Use simulation (Gazebo) clock if true",
    )

    declare_use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value=use_rviz,
        description="Use rvizi if true",
    )

    # Add namespace to robot_localization parameter files
    namespaced_ekf_localization_params = ReplaceString(
        source_file=os.path.join(config_file_dir, "ekf_localization_with_gps.yaml"),
        replacements={"namespace": namespace},
    )

    namespaced_nav2_params = ReplaceString(
        source_file=os.path.join(config_file_dir, "nav2_params.yaml"),
        replacements={"/namespace": ("/", namespace)}, 
    )

    namespaced_nav2_params = RewrittenYaml(
        source_file=namespaced_nav2_params,
        root_key=namespace,
        param_rewrites={
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
        condition=IfCondition(use_rviz),
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
<<<<<<< Updated upstream
=======
            ("imu", "/wit9073can_imu/data"),
>>>>>>> Stashed changes
        ],
    )

    robot_localization_global_node = Node(
        namespace=namespace,
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_filter_node",
        output="screen",
        parameters=[namespaced_ekf_localization_params, {"use_sim_time": use_sim_time}],
        remappings=remapping
        + [
            ("odometry/filtered", "odometry/filtered/global"),
<<<<<<< Updated upstream
=======
            ("imu", "/wit9073can_imu/data"),
>>>>>>> Stashed changes
        ],
    )

    navsat_transform_node = Node(
        namespace=namespace,
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[namespaced_ekf_localization_params, {"use_sim_time": use_sim_time}],
        remappings=remapping
        + [
<<<<<<< Updated upstream
            ("imu/data", "imu"),
            ("gps/fix", "fix"),
            ("odometry/filtered", "odometry/filtered/global"),
=======
            ("imu", "/wit9073can_imu/data"),
            ("gps/fix", "/fix"),
            #("gps/filtered", "/gps/filtered"),
            #("odometry/gps", "/odometry/gps"),
            ("odometry/filtered", "/odometry/filtered/global"),
>>>>>>> Stashed changes
        ],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_use_rviz_arg)

    # Add the commands to the launch description
    ld.add_action(robot_localization_local_node)
    ld.add_action(robot_localization_global_node)
    ld.add_action(navsat_transform_node)  

    ld.add_action(rviz2_node)

    return ld
