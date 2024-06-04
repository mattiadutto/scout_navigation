# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Usage:
#   $ ros2 launch scout_gazebo_sim scout_mini_workshop_world.launch.py use_rviz:=false (pretty slow)
#
#   $ ros2 launch slam_utils realsense_d435i_color_sim.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'base_link',
          'subscribe_depth':True,
          "publish_tf_odom": False,
          'subscribe_odom_info':False,
          'approx_sync':False,
          'wait_imu_to_init':False}]

    remappings=[
          ('imu', '/scout_mini/imu'),
          ('rgb/image', '/color/image_raw'),
          ('rgb/camera_info', '/color/camera_info'),
          ('depth/image', '/depth/image_raw'),
          ("odom", "/scout_mini/odom"),
          ("/tf", "/scout_mini/tf"),
            ("/tf_static", "/scout_mini/tf_static")]

    return LaunchDescription([

        # Nodes to launch       
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings,
           ),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'],
           ),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings,
           ),
        
        # Because of this issue: https://github.com/IntelRealSense/realsense-ros/issues/2564
        # Generate point cloud from not aligned depth
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'approx_sync':False}],
            remappings=[('depth/image',       '/depth/image_raw'),
                        ('depth/camera_info', '/depth/camera_info'),
                        ('cloud',             '/depth/color/points'),
                        ("/tf", "/scout_mini/tf"),
                        ("/tf_static", "/scout_mini/tf_static")],
                       ),
        
        # Generate aligned depth to color camera from the point cloud above       
        Node(
            package='rtabmap_util', executable='pointcloud_to_depthimage', output='screen',
            parameters=[{ 'decimation':2,
                          'fixed_frame_id':'camera_link',
                          'fill_holes_size':1}],
            remappings=[('camera_info', '/color/camera_info'),
                        ('cloud',       '/depth/color/points'),
                        ('image_raw',   '/depth/image_raw'),
                        ("/tf", "/scout_mini/tf"),
                        ("/tf_static", "/scout_mini/tf_static")],
           ),
        
        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/scout_mini/imu'),
                        ("/tf", "/scout_mini/tf"),
                        ("/tf_static", "/scout_mini/tf_static")],
           ),
        
        # The IMU frame is missing in TF tree, add it:
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']),
    ])
