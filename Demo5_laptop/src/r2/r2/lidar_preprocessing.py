"""
ABU Robocon 2026 - Kung Fu Quest
LiDAR Preprocessing — R2 Meihua Forest

Pipeline (per frame):
    Raw Livox PointCloud2
      -> voxel downsample          (reduce point count)
      -> range filter              (world frame — field boundaries relative to start pose)
      -> RANSAC ground removal     (dynamic plane fit, no fixed Z offset)
      -> statistical outlier removal
      -> publish /r2/lidar/preprocessed

Output contains all above-ground objects in range:
    KFS, R1, humans, anything inside the field.
Classification happens downstream in lidar_camera_fusion.py.

Requirements:
    pip install open3d numpy rclpy sensor_msgs
"""

import math
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped


# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

class PreprocessConfig:

    # ── Voxel downsampling ────────────────────────────────────────────────────
    VOXEL_SIZE          = 0.03      # 3 cm — balances detail vs compute

    # ── Field boundary offsets from robot start position (meters) ─────────────
    # X: robot looks 0.8 m behind and 12 m ahead at start
    # Y: robot looks 1.8 m to positive side and 4.2 m to negative side at start
    X_BACK              = 0.8
    X_FRONT             = 12.0
    Y_POS               = 1.8
    Y_NEG               = 3.9

    # ── Z filter (robot frame) ────────────────────────────────────────────────
    Z_MAX               = 2.0       # ignore points above this height
    Z_MIN               = -1.5      # LiDAR at 0.8m height, 15° down tilt

    # ── RANSAC ground removal ─────────────────────────────────────────────────
    RANSAC_DISTANCE_THRESHOLD = 0.03   # 3 cm — points within this of the plane = ground
    RANSAC_N              = 3          # min points to fit a plane
    RANSAC_ITERATIONS     = 100        # more = more robust, slower
    GROUND_Z_MARGIN       = 0.05       # keep points > 5 cm above fitted ground plane

    # ── Statistical outlier removal ───────────────────────────────────────────
    SOR_K_NEIGHBORS     = 20
    SOR_STD_RATIO       = 1.5


# ─────────────────────────────────────────────────────────────────────────────
#  PIPELINE STEPS
# ─────────────────────────────────────────────────────────────────────────────

def ros2_to_open3d(msg: PointCloud2) -> o3d.geometry.PointCloud:
    pts = [
        [p[0], p[1], p[2]]
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    ]
    pcd = o3d.geometry.PointCloud()
    if pts:
        pcd.points = o3d.utility.Vector3dVector(np.array(pts, dtype=np.float32))
    return pcd


def open3d_to_ros2(pcd: o3d.geometry.PointCloud, frame_id: str, stamp) -> PointCloud2:
    pts = np.asarray(pcd.points, dtype=np.float32)
    h = Header()
    h.stamp = stamp
    h.frame_id = frame_id
    return pc2.create_cloud_xyz32(h, pts.tolist())


def range_filter(pcd: o3d.geometry.PointCloud,
                 cfg: PreprocessConfig,
                 r2_x: float, r2_y: float, r2_yaw: float,
                 field_x_min: float, field_x_max: float,
                 field_y_min: float, field_y_max: float) -> o3d.geometry.PointCloud:
    """
    Filter points to the field boundaries (world frame).
    Points are in robot frame — rotate them to world frame using r2_yaw,
    then apply the fixed field boundary box.
    """
    pts = np.asarray(pcd.points)
    if pts.shape[0] == 0:
        return pcd

    # Rotate robot-frame XY into world frame
    cos_y = np.cos(r2_yaw)
    sin_y = np.sin(r2_yaw)
    px = pts[:, 0]
    py = pts[:, 1]
    x_world = r2_x + cos_y * px - sin_y * py
    y_world = r2_y + sin_y * px + cos_y * py

    mask = (
        (x_world >= field_x_min) & (x_world <= field_x_max) &
        (y_world >= field_y_min) & (y_world <= field_y_max) &
        (pts[:, 2] >= cfg.Z_MIN) & (pts[:, 2] <= cfg.Z_MAX)
    )
    out = o3d.geometry.PointCloud()
    out.points = o3d.utility.Vector3dVector(pts[mask])
    return out


def remove_ground_ransac(pcd: o3d.geometry.PointCloud,
                         cfg: PreprocessConfig):
    """
    Fit a plane to the dominant ground surface using RANSAC.
    Returns (above_ground_pcd, ground_pcd, plane_model).
    plane_model = [a, b, c, d]  where ax + by + cz + d = 0
    """
    if len(pcd.points) < cfg.RANSAC_N:
        return pcd, o3d.geometry.PointCloud(), None

    plane_model, inlier_idx = pcd.segment_plane(
        distance_threshold=cfg.RANSAC_DISTANCE_THRESHOLD,
        ransac_n=cfg.RANSAC_N,
        num_iterations=cfg.RANSAC_ITERATIONS
    )

    pts = np.asarray(pcd.points)
    a, b, c, d = plane_model

    # Signed distance of each point from the fitted plane
    norm = np.sqrt(a**2 + b**2 + c**2)
    signed_dist = (a * pts[:, 0] + b * pts[:, 1] + c * pts[:, 2] + d) / norm

    # Keep points that are above the plane by at least GROUND_Z_MARGIN
    # The ground normal points up, so above-ground points have positive signed dist
    if c < 0:
        signed_dist = -signed_dist   # flip if normal points down

    above_mask = signed_dist > cfg.GROUND_Z_MARGIN

    above = o3d.geometry.PointCloud()
    above.points = o3d.utility.Vector3dVector(pts[above_mask])

    ground = o3d.geometry.PointCloud()
    ground.points = o3d.utility.Vector3dVector(pts[~above_mask])

    return above, ground, plane_model


def statistical_outlier_removal(pcd: o3d.geometry.PointCloud,
                                cfg: PreprocessConfig) -> o3d.geometry.PointCloud:
    if len(pcd.points) < cfg.SOR_K_NEIGHBORS:
        return pcd
    clean, _ = pcd.remove_statistical_outlier(cfg.SOR_K_NEIGHBORS, cfg.SOR_STD_RATIO)
    return clean


# ─────────────────────────────────────────────────────────────────────────────
#  FULL PIPELINE
# ─────────────────────────────────────────────────────────────────────────────

def preprocess_pipeline(raw_pcd: o3d.geometry.PointCloud,
                        cfg: PreprocessConfig,
                        r2_x: float, r2_y: float, r2_yaw: float,
                        field_x_min: float, field_x_max: float,
                        field_y_min: float, field_y_max: float,
                        verbose: bool = False) -> dict:

    def log(stage, pcd):
        if verbose:
            print(f"  [{stage:28s}] {len(pcd.points):>6} pts")

    log("Input", raw_pcd)

    # 1. Voxel downsample
    pcd = raw_pcd.voxel_down_sample(cfg.VOXEL_SIZE)
    log("Voxel downsample", pcd)

    if len(pcd.points) == 0:
        return {"preprocessed": pcd, "ground": o3d.geometry.PointCloud(),
                "plane_model": None, "raw_count": len(raw_pcd.points), "final_count": 0}

    # 2. Range filter (world frame field boundaries, rotated by robot orientation)
    pcd = range_filter(pcd, cfg, r2_x, r2_y, r2_yaw,
                       field_x_min, field_x_max, field_y_min, field_y_max)
    log("Range filter", pcd)

    if len(pcd.points) == 0:
        return {"preprocessed": pcd, "ground": o3d.geometry.PointCloud(),
                "plane_model": None, "raw_count": len(raw_pcd.points), "final_count": 0}

    # 3. RANSAC ground removal
    pcd, ground, plane_model = remove_ground_ransac(pcd, cfg)
    log("RANSAC ground removal", pcd)

    if verbose and plane_model is not None:
        a, b, c, d = plane_model
        print(f"    Ground plane: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")

    # 4. Statistical outlier removal
    pcd = statistical_outlier_removal(pcd, cfg)
    log("Statistical outlier removal", pcd)

    return {
        "preprocessed": pcd,
        "ground":       ground,
        "plane_model":  plane_model,
        "raw_count":    len(raw_pcd.points),
        "final_count":  len(pcd.points),
    }


# ─────────────────────────────────────────────────────────────────────────────
#  ROS2 NODE
# ─────────────────────────────────────────────────────────────────────────────

class LidarPreprocessNode(Node):

    def __init__(self):
        super().__init__('lidar_preprocess_node')

        self.declare_parameter('input_topic',  '/livox/lidar')
        self.declare_parameter('output_topic', '/r2/lidar/preprocessed')
        self.declare_parameter('verbose',       False)

        input_topic  = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.verbose = self.get_parameter('verbose').value

        self.cfg = PreprocessConfig()

        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.callback, 10
        )
        self.r2_sub = self.create_subscription(
            PoseStamped, "/r2/pose", self.r2_callback, 10
        )
        self.pub = self.create_publisher(
            PointCloud2, output_topic, 10
        )

        # Current robot pose
        self.r2_x   = None
        self.r2_y   = None
        self.r2_yaw = None

        # Field boundaries in world frame — locked on first pose received
        self.field_x_min = None
        self.field_x_max = None
        self.field_y_min = None
        self.field_y_max = None

        self.get_logger().info(
            f"LidarPreprocessNode ready | in={input_topic} | out={output_topic}"
        )

    def r2_callback(self, msg: PoseStamped):
        self.r2_x = msg.pose.position.x
        self.r2_y = msg.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.orientation
        self.r2_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # Lock field boundaries on first pose
        if self.field_x_min is None:
            self.field_x_min = self.r2_x - self.cfg.X_BACK
            self.field_x_max = self.r2_x + self.cfg.X_FRONT
            self.field_y_min = self.r2_y - self.cfg.Y_NEG
            self.field_y_max = self.r2_y + self.cfg.Y_POS
            self.get_logger().info(
                f"Field boundaries set | "
                f"X:[{self.field_x_min:.2f}, {self.field_x_max:.2f}] "
                f"Y:[{self.field_y_min:.2f}, {self.field_y_max:.2f}]"
            )

    def callback(self, msg: PointCloud2):
        if self.r2_x is None:
            return  # no pose yet

        raw = ros2_to_open3d(msg)
        if len(raw.points) == 0:
            return

        result = preprocess_pipeline(
            raw, self.cfg,
            r2_x=self.r2_x, r2_y=self.r2_y, r2_yaw=self.r2_yaw,
            field_x_min=self.field_x_min, field_x_max=self.field_x_max,
            field_y_min=self.field_y_min, field_y_max=self.field_y_max,
            verbose=self.verbose
        )

        if self.verbose:
            self.get_logger().info(
                f"{result['raw_count']} -> {result['final_count']} pts"
            )

        self.pub.publish(
            open3d_to_ros2(result["preprocessed"], msg.header.frame_id, msg.header.stamp)
        )


# ─────────────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = LidarPreprocessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
