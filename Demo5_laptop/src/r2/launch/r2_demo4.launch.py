from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([

        # LiDAR detection node
        Node(
            package='r2',
            executable='kfs_lidar_node',
            name='kfs_lidar_node',
            output='screen',
            parameters=[{
                'map_path': get_package_share_directory('r2') + '/gamefield.pcd',
                'verbose': False,
            }]
        ),

        # Camera detection node
        Node(
            package='r2',
            executable='kfs_camera_node',
            name='kfs_camera_node',
            output='screen',
        ),

        # Fusion node
        Node(
            package='r2',
            executable='kfs_fusion_node',
            name='kfs_fusion_node',
            output='screen',
            parameters=[{
                'calib_json':   '/home/utmrbc/preprocessed_static/calib.json',
                'camera_yaml':  '/home/utmrbc/camera_calibration.yaml',
                'debug_image':  True,
            }]
        ),

        # Aggregator node
        Node(
            package='aggregator_node',
            executable='aggregator_node',
            name='aggregator_node',
            output='screen',
            parameters=[{
                'can_channel': 'can0',
                'can_bitrate': 1000000,
                'publish_hz':  10.0,
            }]
        ),

        # R1 Localization node
        Node(
            package='r1_localization_node',
            executable='r1_localization_node',
            name='r1_localization_node',
            output='screen',
        ),
    ])