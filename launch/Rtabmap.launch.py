from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_link',
          'odom_frame_id':'odom',
          'odom_tf_linear_variance':0.0005,
          'odom_tf_angular_variance':0.0005,
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'approx_sync':True,
          'sync_queue_size': 15,
          'RGBD/NeighborLinkRefining': 'true',
          'RGBD/ProximityBySpace':     'true',
          'RGBD/ProximityByTime':      'false',
          'RGBD/ProximityPathMaxNeighbors': '15',
          'Reg/Strategy':              '1',
          'Vis/MinInliers':            '10',
          'RGBD/OptimizeFromGraphEnd': 'false',
          'RGBD/OptimizeMaxError':     '3',
          'Reg/Force3DoF':             'true',
          'Grid/FromDepth':            'true',
          'Mem/STMSize':               '50',
          'RGBD/LocalRadius':          '6',
          'Icp/CorrespondenceRatio':   '0.25',
          'Icp/PM':                    'false',
          'Icp/PointToPlane':          'true',
          'Icp/MaxCorrespondenceDistance': '0.1',
          'Icp/VoxelSize':             '0.04'
    }
    
    remappings=[
         ('rgb/image',       '/camera/image_raw'),
         ('depth/image',     '/camera/depth/image_raw'),
         ('rgb/camera_info', '/camera/camera_info'),
         ('scan',            '/scan')]
    
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )

    return LaunchDescription([

        DeclareLaunchArgument('rtabmap_viz',  default_value='true',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.'),

        SetParameter(name='use_sim_time', value=True),

        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[parameters,
              {'rgb_image_transport':'compressed',
               'depth_image_transport':'compressedDepth',
               'approx_sync_max_interval': 0.02}],
            remappings=remappings),
        
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
            
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[parameters],
            remappings=remappings),
        # Node(
        #     package='rviz2', executable='rviz2', name="rviz2", output='screen',
        #     condition=IfCondition(LaunchConfiguration("rviz")),
        #     arguments=["-d", LaunchConfiguration("rviz_cfg")]),
    ])
