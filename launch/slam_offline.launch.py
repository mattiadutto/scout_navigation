###
### Code usable for creating 2D from lidar 2D bag data using SLAM-Toolbox.
### Mattia Dutto
###

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_name = "scout_navigation"

    namespace = LaunchConfiguration("namespace", default="")
    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")

    slam_params_file = LaunchConfiguration(
        "slam_params_file", default="mapper_params_offline.yaml"
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=namespace,
        description="Namespace of the robot for simulation usage.",
    )

    declare_autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the slamtoolbox. "
        "Ignored when use_lifecycle_manager is true.",
    )

    declare_use_lifecycle_manager_arg = DeclareLaunchArgument(
        "use_lifecycle_manager",
        default_value="false",
        description="Enable bond connection during node activation",
    )

    declare_slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=slam_params_file,
        description="Specify the slam toolbox configuration file",
    )

    start_sync_slam_toolbox_node = LifecycleNode(
        parameters=[
            PathJoinSubstitution(
                [get_package_share_directory(pkg_name), "config", slam_params_file]
            ),
            {"use_lifecycle_manager": use_lifecycle_manager},
        ],
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        namespace=namespace,
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_sync_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(
                            start_sync_slam_toolbox_node
                        ),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_autostart_arg)
    ld.add_action(declare_use_lifecycle_manager_arg)
    ld.add_action(declare_slam_params_file_arg)

    ld.add_action(start_sync_slam_toolbox_node)

    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld
