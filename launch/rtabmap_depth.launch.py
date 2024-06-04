# Requirements:
#   Install Turtlebot3 packages
#   Note that we can edit /opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models/turtlebot_waffle/model.sdf 
#     to increase min scan range from 0.12 to 0.2 to avoid having scans 
#     hitting the robot itself
# Example:
#   $ export TURTLEBOT3_MODEL=waffle
#   $ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
#
#   SLAM:
#   $ ros2 launch rtabmap_demos turtlebot3_scan.launch.py
#   OR
#   $ ros2 launch rtabmap_launch rtabmap.launch.py visual_odometry:=false frame_id:=base_footprint subscribe_scan:=true depth:=false approx_sync:=true odom_topic:=/odom args:="-d 'RGBD/NeighborLinkRefining true 'Reg/Strategy 1 'Reg/Force3DoF true 'Grid/RangeMin 0.2" use_sim_time:=true qos:=2
#   $ ros2 run topic_tools relay /rtabmap/map /map
#
#   Navigation (install nav2_bringup package):
#     $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
#     $ ros2 launch nav2_bringup rviz_launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    
    parameters={
        'frame_id':'camera_link', # camera_bottom_screw_frame
        # "target_frame": "base_link",
        'use_sim_time':use_sim_time,
        'subscribe_depth':True,
        'subscribe_rgb':False,
        'subscribe_scan':False,
        'subscribe_scan_cloud':False,
        # 'subscribe_odom_info':True,
        'approx_sync':True,
        'use_action_for_goal':True,
        'qos_depth':qos,
        #'qos_scan':qos,
        'qos_imu':qos,
        'Reg/Strategy':'1',
        'Reg/Force3DoF':'true',
        'RGBD/NeighborLinkRefining':'True',
        'Grid/RangeMin':'0.5',                # ignore laser scan points on the robot itself
        'Grid/GroundIsObstacle':'False',
        'Optimizer/GravitySigma':'0',         # Disable imu constraints (we are already in 2D)
        'wait_for_transform':0.5,             # For RTabViz
        "queue_size":50,
        "confidence":100,
    }
    
    remappings=[
        ("rgb/image", "/color/image_raw"),
        ("rgb/camera_info", "/depth/camera_info"),
        ("depth/image", "/depth/image_raw"),
        ('imu_topic', 'imu')]

    ## Arguments

    arg_use_sim_time = DeclareLaunchArgument('use_sim_time', 
        default_value='true', 
        description='Use simulation (Gazebo) clock if true')

    arg_qos = DeclareLaunchArgument('qos', 
        default_value='2',
        description='QoS used for input sensor topics')

    arg_localization = DeclareLaunchArgument('localization', 
        default_value='false',
        description='Launch in localization mode.')

    ## Nodes
    node_rtabmap_slam = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam', 
        executable='rtabmap', 
        output='screen',
        parameters=[parameters],
        namespace="scout_mini",
        remappings=remappings,
        arguments=['-d'] # This will delete the previous database (~/.ros/rtabmap.db)
    )

    node_rtabmap_localization = Node(
        namespace="scout_mini",
        condition=IfCondition(localization),
        package='rtabmap_slam', 
        executable='rtabmap', 
        output='screen',
        parameters=[parameters,
            {'Mem/IncrementalMemory':'False',
            'Mem/InitWMWithAllNodes':'True'}],
        remappings=remappings
    )

    node_rtabmap_rviz = Node(
        package='rtabmap_viz', 
        executable='rtabmap_viz', 
        output='screen',
        parameters=[parameters],
        namespace="scout_mini",
        remappings=remappings
    )

    ld = LaunchDescription()

    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_qos)
    ld.add_action(arg_localization)
    ld.add_action(node_rtabmap_slam)
    ld.add_action(node_rtabmap_localization)
    ld.add_action(node_rtabmap_rviz)

    return ld