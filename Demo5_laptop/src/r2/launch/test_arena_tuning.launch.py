"""
Arena tuning launch — no camera required.

Brings up:
  1. r2_localization_node   — reads GLIM map→imu TF, publishes /r2/pose
  2. lidar_preprocess_node  — raw Livox → /r2/lidar/preprocessed
  3. kfs_fusion_node        — applies tuning offset, publishes
                              /r2/rack_markers  (rviz spheres at slot positions)
                              /r2/tuning_state  (JSON echo of offsets + slot XYZs)
                              /r2/arena/status  (empty until camera detections arrive)

Verify in rviz:
  - Add MarkerArray → /r2/rack_markers
  - Fixed frame: map
  - Confirm spheres S1–S9 land on the physical rack

Tune by adjusting the four offset args below and re-launching.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Tuning offsets — adjust these to match your retry-zone → nominal delta ─
    offset_x   = DeclareLaunchArgument('offset_x',   default_value='10.0',  description='X translation offset (m)')
    offset_y   = DeclareLaunchArgument('offset_y',   default_value='3.2',   description='Y translation offset (m)')
    offset_z   = DeclareLaunchArgument('offset_z',   default_value='0.45',  description='Z height offset (m)')
    offset_yaw = DeclareLaunchArgument('offset_yaw', default_value='90.0',  description='Yaw offset (degrees)')
    playing_as_red = DeclareLaunchArgument('playing_as_red', default_value='true', description='true = red side, false = blue side')

    return LaunchDescription([
        offset_x,
        offset_y,
        offset_z,
        offset_yaw,
        playing_as_red,

        # ── 1. R2 localization ─────────────────────────────────────────────────
        # Reads map→imu TF published by GLIM, outputs /r2/pose
        Node(
            package='r2',
            executable='r2_localization_node',
            name='r2_localization_node',
            output='screen',
        ),

        # ── 2. LiDAR preprocessing ─────────────────────────────────────────────
        # Raw Livox (/livox/lidar) → voxel + range + ground removal → /r2/lidar/preprocessed
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

        # ── 3. LiDAR-camera fusion (tuning mode — no camera needed) ────────────
        # Publishes /r2/rack_markers and /r2/tuning_state at 1 Hz.
        # Camera detections are optional — slot markers appear immediately.
        Node(
            package='r2',
            executable='kfs_fusion_node',
            name='kfs_fusion_node',
            output='screen',
            parameters=[{
                'calib_json':               '/home/utmrbc/preprocessed_static/calib.json',
                'camera_yaml':              '/home/utmrbc/camera_calibration.yaml',
                'playing_as_red':           LaunchConfiguration('playing_as_red'),
                'debug_image':              False,
                'tuning_on_arena':          True,
                'arena_origin_offset_x':    LaunchConfiguration('offset_x'),
                'arena_origin_offset_y':    LaunchConfiguration('offset_y'),
                'arena_origin_offset_z':    LaunchConfiguration('offset_z'),
                'arena_origin_offset_yaw':  LaunchConfiguration('offset_yaw'),
            }],
        ),

    ])
