import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_name = "scout_navigation"
    
    namespace = LaunchConfiguration("namespace", default="/scout_mini")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    slam_params_file = LaunchConfiguration("slam_params_file", default="mapper_params_online_async.yaml")

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace", 
        default_value=namespace,
        description="Namespace of the robot for simulation usage."
    )
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="true", 
        description="Use simulation/Gazebo clock"
    )
    
    slam_params_file_path = PathJoinSubstitution(
        [get_package_share_directory(pkg_name), "config", slam_params_file]
    )

    declare_slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=slam_params_file_path,
        description="Name of the yaml parameters file of slam_toolbox node in mapping mode",
    )

    async_slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file_path, {"use_sim_time": use_sim_time}],
        remappings=[("/scan", f"{namespace}/scan"),
                    ("/tf", f"{namespace}/tf"), 
                    ("/tf_static", f"{namespace}/tf_static")],
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_slam_params_file_arg)
    
    ld.add_action(async_slam_toolbox_node)

    return ld
