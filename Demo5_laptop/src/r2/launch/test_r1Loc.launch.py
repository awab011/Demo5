from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # R2 localization — reads map→imu TF from GLIM, publishes /r2/pose
        Node(
            package='r2',
            executable='r2_localization_node',
            name='r2_localization_node',
            output='screen',
        ),

        # LiDAR preprocessing — raw Livox → /r2/lidar/preprocessed
        Node(
            package='r2',
            executable='lidar_preprocess_node',
            name='lidar_preprocess_node',
            output='screen',
            parameters=[{
                'input_topic':  '/livox/lidar',
                'output_topic': '/r2/lidar/preprocessed',
                'verbose':      False,
            }],
        ),

        # R1 localization — clusters preprocessed cloud, publishes /r1/position
        Node(
            package='r2',
            executable='r1_localization_node',
            name='r1_localization_node',
            output='screen',
        ),

    ])
