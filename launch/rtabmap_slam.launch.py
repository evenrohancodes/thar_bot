import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Set topic names based on your simulation
    depth_topic = LaunchConfiguration('depth_topic', default='/camera/depth/image_raw')
    rgb_topic = LaunchConfiguration('rgb_topic', default='/camera/image_raw')
    camera_info_topic = LaunchConfiguration('camera_info_topic', default='/camera/camera_info')
    # pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/camera/points')

    declare_depth_topic = DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw')
    declare_rgb_topic = DeclareLaunchArgument('rgb_topic', default_value='/camera/image_raw')
    declare_camera_info_topic = DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info')
    # declare_pointcloud_topic = DeclareLaunchArgument('pointcloud_topic', default_value='/camera/points')

    # RTAB-Map node for SLAM
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'camera_link',
            'odom_frame_id': 'odom',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,  # Set True if using LiDAR
            'queue_size': 10,
            'rgb_topic': rgb_topic,
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic,
            'approx_sync': True,
            'topic_queue_size': 10,
            'sync_queue_size': 10,
            'use_sim_time': True,
            'qos': 2 
        }],
        remappings=[
            ('/rtabmap/rgb/image', rgb_topic),
            ('/rtabmap/depth/image', depth_topic),
            ('/rtabmap/rgb/camera_info', camera_info_topic),
            # ('/rtabmap/cloud_map', pointcloud_topic)
        ]
    )

    # RViz for visualization
    # rviz_config_file = os.path.join(get_package_share_directory("rtabmap_slam"), "launch", "rtabmap.rviz")
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", rviz_config_file]
    # )

    return LaunchDescription([
        declare_depth_topic,
        declare_rgb_topic,
        declare_camera_info_topic,
        # declare_pointcloud_topic,
        rtabmap_node,
        
    ])
