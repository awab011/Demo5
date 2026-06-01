from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('playing_as',             default_value='Red',     description="'Red' or 'Blue'"),
        DeclareLaunchArgument('starting_zone',          default_value='Default', description="'Default' or 'Arena'"),
        DeclareLaunchArgument('robot_facing_spearhead', default_value='true',    description='true or false'),

        # R2 localization — reads map→imu TF from GLIM, publishes /r2/pose
        Node(
            package='r2',
            executable='r2_localization_node',
            name='r2_localization_node',
            output='screen',
            parameters=[{
                'starting_zone':          LaunchConfiguration('starting_zone'),
                'use_world_map':          True,
                'robot_facing_spearhead': LaunchConfiguration('robot_facing_spearhead'),
                'playing_as':             LaunchConfiguration('playing_as'),
            }],
        ),

        # R2 camera detection — detects KFS in camera feed, publishes /kfs/detections
        Node(
            package='r2',
            executable='kfs_camera_node',
            name='kfs_camera_node',
            output='screen',
        ),

        # Fusion node — fuses lidar + camera detections + pose
        Node(
            package='r2',
            executable='kfs_fusion_node',
            name='kfs_fusion_node',
            output='screen',
            parameters=[{
                'calib_json':        '/home/utmrbc/preprocessed_static/calib.json',
                'camera_yaml':       '/home/utmrbc/camera_calibration.yaml',
                'debug_image':       True,
                'max_cloud_age_ms':  300.0,
            }],
        ),
    ])