"""
ABU Robocon 2026 - Kung Fu Quest
LiDAR-Camera Fusion Node — R2 Meihua Forest

Replaces kfs_lidar_detection.py + old fusion node into one step.

Pipeline (10 Hz):
    1. Project preprocessed LiDAR cloud onto camera image
    2. For each camera KFS detection bbox:
         - collect LiDAR points projecting inside bbox
         - filter to nearest-depth cluster (removes background behind KFS)
         - centroid → 3D KFS position (robot frame → world frame via odometry)
         - assign to Meihua Forest block (forest mode) or Tic-Tac-Toe slot (arena mode)
    3. Publish confirmed grid / arena status, debug image

Subscribes:
    /r2/lidar/preprocessed    PointCloud2
    /kfs/detections           Detection2DArray
    /camera1/image_raw        Image
    /r2/pose                  PoseStamped

Publishes:
    /r2/forest/confirmed_grid  String (JSON)   — forest mode
    /r2/arena/status           String (JSON)   — arena mode
    /kfs/fusion_debug          Image
"""

import json
import math
import os
import threading
from collections import deque

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header, String
from vision_msgs.msg import Detection2DArray
from r2_msgs.msg import ArenaStatus, KfsSlot
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2


# –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
#  CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
CLASS_ID_TO_WHOS_KFS = {
    0: "TEAM_KFS",
    1: "OPP_KFS",
}
CLASS_ID_TO_KFS_TYPE = {
    0: "Fake_KFS",
    1: "R1_KFS",
    2: "R2_KFS",
}
CLASS_ID_TO_COLOR = {
    0: (0,   0,   255),   # red    — Fake_KFS
    1: (0,   128, 255),   # orange — R1_KFS
    2: (0,   255, 0  ),   # green  — R2_KFS
}
MAIN_CLASS_ID_TO_LABEL = {
    0: "BLUE",
    1: "RED",
}


# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

class FusionConfig:

    # ── Meihua Forest block grid (world frame, meters) ────────────────────────
    # Set BLOCK_1_X/Y to the world-frame position of block 1 (entry block).
    # Measured from the odometry origin (robot start position).
    BLOCK_1_X        = 2.4
    BLOCK_1_Y        = -3.0
    BLOCK_SPACING_X  = 1.20   # meters between block centres (forward axis)
    BLOCK_SPACING_Y  = 1.20   # meters between block centres (lateral axis)
    BLOCK_TOP_Z      = 0.40   # block top height above floor (meters)

    # # ── Tik tac Teo slots grid (world frame, meters) ────────────────────────
    # Set SLOT1_X/Y to the world-frame position of block 1 (entry block).
    # Measured from the odometry origin (robot start position).
    SLOT_1_X        =  9.8
    SLOT_1_Y        =  1.665 
    SLOT_SPACING_Z  =  0.54   # meters between SLOT centres (forward axis)
    SLOT_SPACING_Y  =  0.54   # meters between SLOT centres (lateral axis)
    FIRST_ROW_Z     =  0.4    # first row height above floor (meters)
    MID_ROW_Z       =  0.94   # middle row height above floor (meters)
    TOP_ROW_Z       =  1.48   # top row height above floor (meters)

    # Max XY distance from a block centre to assign a KFS to it
    # Must be less than half the block spacing (0.6m) to avoid ambiguity
    ASSIGNMENT_MAX_DIST_MFF   = 0.40
    ASSIGNMENT_MAX_DIST_ARENA = 0.20

    # Extra pixels added to each side of a detection bbox when collecting LiDAR points.
    # Compensates for small calibration offsets — increase if top/edge detections miss points.
    BBOX_MARGIN_PX = 20

    # ── LiDAR points inside bbox — depth filtering ────────────────────────────
    # Keep only points within this margin beyond the nearest point in the bbox.
    # Filters out background objects behind the KFS.
    KFS_DEPTH_MARGIN = 0.40   # meters
    KFS_MIN_POINTS   = 1      # minimum points in bbox to count as a detection

    # ── Grid rotation (set by tuning block; identity by default) ─────────────
    GRID_COS_T = 1.0
    GRID_SIN_T = 0.0

    # ── Timestamp sync ────────────────────────────────────────────────────────
    SYNC_TOLERANCE_MS = 150.0
    
    
# –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
# KFS POSITION COMPUTING
# ─────────────────────────────────────────────────────────────────────––––––––
def compute_kfs_position(kfs_distance, zone: str, playing_as_red: bool, world_xyz: np.ndarray) -> tuple:
    

    kfs_pisition =  kfs_distance['x_m'] + world_xyz['x_m']



class MeihuaForestGrid:
    """
    3x4 block grid layout:

        EXIT  [10][11][12]   row 3
              [ 7][ 8][ 9]   row 2
              [ 4][ 5][ 6]   row 1
        ENTRY [ 1][ 2][ 3]   row 0
    """

    BLOCK_LAYOUT = {
         1: (0, 0),  2: (1, 0),  3: (2, 0),
         4: (0, 1),  5: (1, 1),  6: (2, 1),
         7: (0, 2),  8: (1, 2),  9: (2, 2),
        10: (0, 3), 11: (1, 3), 12: (2, 3),
    }

    ADJACENCY = {
         1: [2, 4],       2: [1, 3, 5],      3: [2, 6],
         4: [1, 5, 7],    5: [2, 4, 6, 8],   6: [3, 5, 9],
         7: [4, 8, 10],   8: [5, 7, 9, 11],  9: [6, 8, 12],
        10: [7, 11],      11: [8, 10, 12],   12: [9, 11],
    }

    def __init__(self, cfg: FusionConfig, playing_as_red: bool = True):
        self.cfg = cfg
        self.mirror = 1.0 if playing_as_red else -1.0
        self.block_positions = self._compute_positions()

    def _compute_positions(self) -> dict:
        positions = {}
        c = self.cfg
        m = self.mirror
        for block_num, (col, row) in self.BLOCK_LAYOUT.items():
            dx = col * c.BLOCK_SPACING_X
            dy = row * c.BLOCK_SPACING_Y
            x = c.BLOCK_1_X + c.GRID_COS_T * dx - c.GRID_SIN_T * dy
            y = m * (c.BLOCK_1_Y + c.GRID_SIN_T * dx + c.GRID_COS_T * dy)
            positions[block_num] = np.array([x, y, c.BLOCK_TOP_Z])
        return positions

    def nearest_block(self, world_xyz: np.ndarray) -> tuple:
        best_block, best_dist = None, float('inf')
        for block_num, pos in self.block_positions.items():
            dist = float(np.linalg.norm(world_xyz[:2] - pos[:2]))
            if dist < best_dist:
                best_dist = dist
                best_block = block_num
        return best_block, best_dist

# ─────────────────────────────────────────────────────────────────────────────
#  TIC TAC TOE RACK GRID
# ─────────────────────────────────────────────────────────────────────────────
class TikTacTeoRackGrid:
    """
    3x3 slot grid layout:

        [7][8][9]   row 3
        [4][5][6]   row 2
        [1][2][3]   row 1
    """

    SLOT_LAYOUT = {
         1: (0, 0),  2: (1, 0),  3: (2, 0),
         4: (0, 1),  5: (1, 1),  6: (2, 1),
         7: (0, 2),  8: (1, 2),  9: (2, 2),
    }

    def __init__(self, cfg: FusionConfig, playing_as_red: bool = True):
        self.cfg = cfg
        self.mirror = 1.0 if playing_as_red else -1.0
        self.slot_positions = self._compute_positions()

    def _compute_positions(self) -> dict:
        positions = {}
        c = self.cfg
        m = self.mirror
        for slot_num, (col, row) in self.SLOT_LAYOUT.items():
            dx = col * c.SLOT_SPACING_Z   # column step — feeds Y axis
            dy = row * c.SLOT_SPACING_Y   # row step    — feeds X axis
            x = c.SLOT_1_X + c.GRID_COS_T * dy - c.GRID_SIN_T * dx
            y = m * (c.SLOT_1_Y + c.GRID_SIN_T * dy + c.GRID_COS_T * dx)
            z = [c.FIRST_ROW_Z, c.MID_ROW_Z, c.TOP_ROW_Z][row]
            positions[slot_num] = np.array([x, y, z])
        return positions
 
    def nearest_slot(self, world_xyz: np.ndarray) -> tuple:
        best_slot, best_dist = None, float('inf')
        for slot_num, pos in self.slot_positions.items():
            dist = float(np.linalg.norm(world_xyz - pos))
            if dist < best_dist:
                best_dist = dist
                best_slot = slot_num
        return best_slot, best_dist

    def team_or_opp(self, playing_as_red:bool, cls:int ):
        if playing_as_red:
            return "TEAM_KFS" if cls == 1 else "OPP_KFS"
        else:
            return "OPP_KFS" if cls == 1 else "TEAM_KFS"




# ─────────────────────────────────────────────────────────────────────────────
#  CALIBRATION LOADING
# ─────────────────────────────────────────────────────────────────────────────

# Latest calibration result (calib_bag_static_new4) — used as default fallback.
_DEFAULT_R = np.array([
    [-0.0502377,  0.287678,    0.956409 ],
    [-0.998641,  -0.00118164, -0.0521007],
    [-0.0138581, -0.957726,   0.287347  ],
], dtype=np.float64)
_DEFAULT_T = np.array([0.106283, 0.901792, 0.74507], dtype=np.float64)


def load_extrinsic(path: str):
    with open(path, 'r') as f:
        data = json.load(f)
    results = data.get('results', {})

    # Prefer 4x4 matrix format (T_lidar_camera_matrix) — exact, no quaternion rounding
    if 'T_lidar_camera_matrix' in results:
        M = np.array(results['T_lidar_camera_matrix'], dtype=np.float64)
        return M[:3, :3], M[:3, 3]

    # Legacy quaternion format (T_lidar_camera: [qx, qy, qz, qw, tx, ty, tz])
    if 'T_lidar_camera' in results:
        T = results['T_lidar_camera']
        qx, qy, qz, qw = T[0], T[1], T[2], T[3]
        tx, ty, tz      = T[4], T[5], T[6]
        R = np.array([
            [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qz*qw),  2*(qx*qz + qy*qw)],
            [    2*(qx*qy + qz*qw),  1 - 2*(qx**2 + qz**2),  2*(qy*qz - qx*qw)],
            [    2*(qx*qz - qy*qw),  2*(qy*qz + qx*qw),  1 - 2*(qx**2 + qy**2)],
        ], dtype=np.float64)
        return R, np.array([tx, ty, tz], dtype=np.float64)

    # No calibration result in file — use hardcoded default
    import warnings
    warnings.warn(
        "calib.json has no 'T_lidar_camera_matrix' or 'T_lidar_camera' key — "
        "using hardcoded default (calib_bag_static_new4). "
        "Add T_lidar_camera_matrix to calib.json results to suppress this.",
        stacklevel=2,
    )
    return _DEFAULT_R, _DEFAULT_T


def load_camera_intrinsics(path: str):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    K = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(data['distortion_coefficients']['data'], dtype=np.float64)
    return K, D


# ─────────────────────────────────────────────────────────────────────────────
#  ODOMETRY → 4x4 TRANSFORM
# ─────────────────────────────────────────────────────────────────────────────

def pose_to_transform(pose: PoseStamped) -> np.ndarray:
    """Extract 4x4 robot-to-world transform from a PoseStamped message."""
    p = pose.pose.position
    q = pose.pose.orientation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    R = np.array([
        [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qz*qw),  2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw),  1 - 2*(qx**2 + qz**2),  2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),  2*(qy*qz + qx*qw),  1 - 2*(qx**2 + qy**2)],
    ], dtype=np.float64)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = [p.x, p.y, p.z]
    return T


def transform_points(pts: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to an (N,3) point array."""
    n = pts.shape[0]
    pts_h = np.hstack([pts, np.ones((n, 1))])
    return (T @ pts_h.T).T[:, :3]


# ─────────────────────────────────────────────────────────────────────────────
#  LIDAR → IMAGE PROJECTION
# ─────────────────────────────────────────────────────────────────────────────

def project_lidar_to_image(points_lidar: np.ndarray,
                            R: np.ndarray, t: np.ndarray,
                            K: np.ndarray, D: np.ndarray):
    """
    Project LiDAR points into the camera image.
    Returns (pixels (N,2), valid_mask (N,)) where valid = in front of camera.
    """
    if len(points_lidar) == 0:
        return np.zeros((0, 2)), np.zeros(0, dtype=bool)

    # LiDAR frame → camera frame  (inverse of stored T_lidar_camera)
    R_inv = R.T
    t_inv = -R_inv @ t
    pts_cam = (R_inv @ points_lidar.T).T + t_inv

    valid = pts_cam[:, 2] > 0.1
    pixels = np.zeros((len(points_lidar), 2))

    if np.any(valid):
        proj, _ = cv2.projectPoints(
            pts_cam[valid], np.zeros(3), np.zeros(3), K, D
        )
        pixels[valid] = proj.reshape(-1, 2)

    return pixels, valid


# ─────────────────────────────────────────────────────────────────────────────
#  POINTS INSIDE BOUNDING BOX + DEPTH FILTER
# ─────────────────────────────────────────────────────────────────────────────

def points_in_bbox(pts: np.ndarray, pixels: np.ndarray,
                   valid: np.ndarray, det, cfg: FusionConfig,
                   img_w: int, img_h: int) -> np.ndarray:
    """
    Return the subset of pts whose projected pixel falls inside det's bbox,
    filtered to points within KFS_DEPTH_MARGIN of the bbox-center point's depth.
    """
    cx = det.bbox.center.position.x
    cy = det.bbox.center.position.y
    hw = det.bbox.size_x / 2 + cfg.BBOX_MARGIN_PX
    hh = det.bbox.size_y / 2 + cfg.BBOX_MARGIN_PX

    in_bbox = (
        valid &
        (pixels[:, 0] >= cx - hw) & (pixels[:, 0] <= cx + hw) &
        (pixels[:, 1] >= cy - hh) & (pixels[:, 1] <= cy + hh) &
        (pixels[:, 0] >= 0) & (pixels[:, 0] < img_w) &
        (pixels[:, 1] >= 0) & (pixels[:, 1] < img_h)
    )

    bbox_pts = pts[in_bbox]
    if len(bbox_pts) < cfg.KFS_MIN_POINTS:
        return np.empty((0, 3))

    # Depth filter: anchor on the closest (minimum depth) point — stable across frames
    depths = np.linalg.norm(bbox_pts, axis=1)
    center_depth = depths.min()
    near_mask = np.abs(depths - center_depth) <= cfg.KFS_DEPTH_MARGIN

    return bbox_pts[near_mask]


# ─────────────────────────────────────────────────────────────────────────────
#  ROS2 NODE
# ─────────────────────────────────────────────────────────────────────────────

class LidarCameraFusionNode(Node):

    def __init__(self):
        super().__init__('lidar_camera_fusion_node')

        self.declare_parameter('calib_json',    '~/preprocessed_static/calib.json')
        self.declare_parameter('camera_yaml',   '/home/utmrbc/camera_calibration.yaml')
        self.declare_parameter('image_width',   1280)
        self.declare_parameter('image_height',  720)
        self.declare_parameter('sync_tolerance_ms', 150.0)
        self.declare_parameter('max_cloud_age_ms', 50.0)
        self.declare_parameter('debug_image',   True)
        self.declare_parameter('in_arena',     False)
        self.declare_parameter('tuning_on_arena',         True)

        calib_json  = os.path.expanduser(self.get_parameter('calib_json').value)
        camera_yaml = os.path.expanduser(self.get_parameter('camera_yaml').value)
        self.img_w  = self.get_parameter('image_width').value
        self.img_h  = self.get_parameter('image_height').value
        self.sync_tol = self.get_parameter('sync_tolerance_ms').value * 1e6  # ns
        self.max_cloud_age = self.get_parameter('max_cloud_age_ms').value * 1e6  # ns
        self.debug  = self.get_parameter('debug_image').value
        self.in_arena = self.get_parameter('in_arena').value
        self.playing_as_red = True  # updated from /r2/pose frame_id on first message
        self._tuning_on_arena = self.get_parameter('tuning_on_arena').value
        if self._tuning_on_arena:
            self.in_arena = True   # tuning always exercises the arena pipeline

        try:
            self.R, self.t = load_extrinsic(calib_json)
            self.K, self.D = load_camera_intrinsics(camera_yaml)
            self.get_logger().info(
                f"Calibration loaded\n"
                f"  extrinsic : {calib_json}\n"
                f"  intrinsic : {camera_yaml}\n"
                f"  R[0]      : {self.R[0]}\n"
                f"  t         : {self.t}\n"
                f"  K(fx,fy)  : {self.K[0,0]:.2f}, {self.K[1,1]:.2f}   "
                f"  cx,cy: {self.K[0,2]:.2f}, {self.K[1,2]:.2f}"
            )
        except Exception as e:
            if self._tuning_on_arena:
                self.R = np.eye(3)
                self.t = np.zeros(3)
                self.K = np.eye(3)
                self.D = np.zeros(5)
                self.get_logger().warn(f"Calibration not loaded ({e}) — OK in tuning mode, camera disabled")
            else:
                raise

        self.cfg = FusionConfig()

        self.get_logger().info(
            f'SLOT_1=({self.cfg.SLOT_1_X:.3f}, {self.cfg.SLOT_1_Y:.3f})  '
            f'BLOCK_1=({self.cfg.BLOCK_1_X:.3f}, {self.cfg.BLOCK_1_Y:.3f})  '
            f'Z=({self.cfg.FIRST_ROW_Z:.3f}, {self.cfg.MID_ROW_Z:.3f}, {self.cfg.TOP_ROW_Z:.3f})'
        )

        self.rack = TikTacTeoRackGrid(self.cfg, self.playing_as_red)
        self.grid = MeihuaForestGrid(self.cfg, self.playing_as_red)

        self.lock               = threading.Lock()
        self.latest_cloud       = None
        self.latest_cloud_ts    = None
        self.latest_dets        = None
        self.latest_dets_ts     = None
        self.latest_image       = None
        self.latest_pose        = None
        self._last_valid_dets   = None
        self.bridge             = CvBridge()
        self._last_stale_warn   = 0.0
        self._centroid_ma: dict = {}   # track_id → deque of raw centroids (robot frame)
        self._ma_window = 70

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(PointCloud2,     '/livox/lidar',
                                 self._cloud_cb,  qos_sensor)
        self.create_subscription(Detection2DArray,'/kfs/detections',
                                 self._dets_cb,   qos_reliable)
        self.create_subscription(Image,           '/camera1/image_raw',
                                 self._image_cb,  qos_sensor)
        self.create_subscription(PoseStamped,     '/r2/map_pose',
                                 self._pose_cb,   qos_sensor)
        

        self.pub_confirmed = self.create_publisher(String,      '/r2/forest/confirmed_grid', 10)
        self.arena_status  = self.create_publisher(ArenaStatus, '/r2/arena/status',          10)
        self.pub_debug     = self.create_publisher(Image,       '/kfs/fusion_debug',         10)
        self.pub_distances = self.create_publisher(String,      '/kfs/distances',            10)

        if self._tuning_on_arena:
            self._pub_markers     = self.create_publisher(MarkerArray, '/r2/rack_markers',   10)
            self.create_timer(1.0, self._publish_tuning_state)

        self.create_timer(0.1, self._fuse)
        self.get_logger().info("LidarCameraFusionNode ready.")

    # ── Tuning state publisher ────────────────────────────────────────────────

    def _publish_tuning_state(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        # Row colours: bottom=green, middle=blue, top=red
        row_rgba = {
            0: (0.2, 1.0, 0.2, 0.9),
            1: (0.2, 0.5, 1.0, 0.9),
            2: (1.0, 0.2, 0.2, 0.9),
        }

        slot_data = {}
        for slot_num, pos in self.rack.slot_positions.items():
            col, row = self.rack.SLOT_LAYOUT[slot_num]
            r, g, b, a = row_rgba[row]
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])

            # Sphere marker
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp    = now
            sphere.ns              = 'rack_slots'
            sphere.id              = slot_num
            sphere.type            = Marker.SPHERE
            sphere.action          = Marker.ADD
            sphere.pose.position   = Point(x=x, y=y, z=z)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.10
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = r, g, b, a
            sphere.lifetime.sec = 2
            markers.markers.append(sphere)

            # Text label
            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp    = now
            label.ns              = 'rack_labels'
            label.id              = slot_num + 100
            label.type            = Marker.TEXT_VIEW_FACING
            label.action          = Marker.ADD
            label.pose.position   = Point(x=x, y=y, z=z + 0.15)
            label.pose.orientation.w = 1.0
            label.scale.z         = 0.08
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text            = f'S{slot_num}\n({x:.2f},{y:.2f},{z:.2f})'
            label.lifetime.sec    = 2
            markers.markers.append(label)

        self._pub_markers.publish(markers)


    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cloud_cb(self, msg: PointCloud2):
        structured = pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        pts = np.stack([structured['x'], structured['y'], structured['z']], axis=-1).astype(np.float64)

        # Fast numpy filter — replaces the entire preprocessing node:
        #   drop floor/ceiling (Z) and points beyond field range (X)
        #   Ground points won't land inside any camera bbox so RANSAC/SOR unnecessary
        if len(pts) > 0:
            mask = (
                (pts[:, 2] > 0.05)   &   # above ground (robot frame)
                (pts[:, 2] < 6.0)    &   # below ceiling
                (pts[:, 0] > -1.0)   &   # not behind robot
                (pts[:, 0] < 12.0)       # within field
            )
            pts = pts[mask]

        ts = Time.from_msg(msg.header.stamp).nanoseconds
        with self.lock:
            self.latest_cloud    = pts
            self.latest_cloud_ts = ts

    def _dets_cb(self, msg: Detection2DArray):
        ts = Time.from_msg(msg.header.stamp).nanoseconds
        with self.lock:
            self.latest_dets    = msg
            self.latest_dets_ts = ts

    def _image_cb(self, msg: Image):
        with self.lock:
            self.latest_image = msg

    def _pose_cb(self, msg: PoseStamped):
        frame = msg.header.frame_id          # 'map_red' or 'map_blue'
        playing_as_red = 'red' in frame.lower()
        with self.lock:
            self.latest_pose = msg
            if playing_as_red != self.playing_as_red:
                self.playing_as_red = playing_as_red
                self.rack = TikTacTeoRackGrid(self.cfg, playing_as_red)
                self.grid = MeihuaForestGrid(self.cfg, playing_as_red)
                self.get_logger().info(
                    f"Team color updated from pose frame_id: {'RED' if playing_as_red else 'BLUE'}"
                )

    # ── Main fusion loop ──────────────────────────────────────────────────────

    def _fuse(self):
        with self.lock:
            pts        = self.latest_cloud
            cloud_ts   = self.latest_cloud_ts
            dets       = self.latest_dets
            dets_ts    = self.latest_dets_ts
            image_msg  = self.latest_image
            pose       = self.latest_pose

        if pts is None or len(pts) == 0:
            return

        # Drop stale clouds — upstream sensor may have died
        now_ns = self.get_clock().now().nanoseconds
        if cloud_ts is not None and (now_ns - cloud_ts) > self.max_cloud_age:
            now_sec = now_ns / 1e9
            if now_sec - self._last_stale_warn > 5.0:
                self.get_logger().warn(
                    f"Stale cloud ({(now_ns - cloud_ts)/1e6:.0f}ms old) — "
                    f"preprocessing too slow? (limit={self.max_cloud_age/1e6:.0f}ms)"
                )
                self._last_stale_warn = now_sec
            return

        # Robot-to-world transform from /r2/pose (map → imu via TF)
        robot_T = pose_to_transform(pose) if pose is not None else np.eye(4)

        # Project all points onto camera
        pixels, valid = project_lidar_to_image(pts, self.R, self.t, self.K, self.D)

        # Check timestamp sync for camera detections; fall back to last valid if sync fails
        use_dets = None
        if dets is not None and cloud_ts is not None and dets_ts is not None:
            if abs(cloud_ts - dets_ts) < self.sync_tol:
                use_dets = dets
                self._last_valid_dets = dets
            else:
                use_dets = self._last_valid_dets
                self.get_logger().debug(
                    f"Timestamp gap: {abs(cloud_ts - dets_ts)/1e6:.1f}ms — using last valid dets"
                )

        # ── KFS detection via camera bounding boxes ───────────────────────────
        confirmed_grid = {b: None for b in self.grid.BLOCK_LAYOUT}
        confirmed_rack = {r: None for r in self.rack.SLOT_LAYOUT}
        kfs_distances  = []

        if use_dets is not None:
            for det in use_dets.detections:
                if not det.results:
                    continue

                # results[0] = auth model → Fake/R1/R2 (type)
                # results[1] = main model → blue/red  (team colour)
                type_id  = int(det.results[0].hypothesis.class_id)
                team_id  = int(det.results[1].hypothesis.class_id) if len(det.results) > 1 else -1
                kfs_type = CLASS_ID_TO_KFS_TYPE.get(type_id, 'UNKNOWN')
                whos_kfs = self.rack.team_or_opp(self.playing_as_red, team_id)

                conf     = float(det.results[0].hypothesis.score)

                bbox_pts = points_in_bbox(
                    pts, pixels, valid, det, self.cfg, self.img_w, self.img_h
                )
                if len(bbox_pts) == 0:
                    continue

                # Drop points below 0.1 m above floor (sensor at 0.8 m → floor at z≈-0.8)
                bbox_pts = bbox_pts[bbox_pts[:, 2] > -0.70]
                if len(bbox_pts) == 0:
                    continue

                # 3D centroid in robot frame — median over points (robust to edge outliers),
                # then median over the history window (robust to bbox jitter)
                raw_centroid = np.median(bbox_pts, axis=0)
                tid = str(det.id)
                if tid not in self._centroid_ma:
                    self._centroid_ma[tid] = deque(maxlen=self._ma_window)

                self._centroid_ma[tid].append(raw_centroid)
                centroid_robot = np.median(self._centroid_ma[tid], axis=0)
                distance_m     = float(np.linalg.norm(centroid_robot))
                centroid_world = transform_points(
                    centroid_robot.reshape(1, 3), robot_T
                )[0]
                centroid_world[2] -= 0.70  # sensor mounted 800 mm above ground

                robot_map_x = float(robot_T[0, 3])
                robot_map_y = float(robot_T[1, 3])
                kfs_map_x   = round(robot_map_x - float(centroid_robot[1]), 3)
                kfs_map_y   = round(robot_map_y + float(centroid_robot[0]), 3)

                kfs_distances.append({
                    'track_id':   det.id,
                    'distance_m': round(distance_m, 3),
                    'x_m':        kfs_map_x,
                    'y_m':        float(centroid_robot[1]),
                    'z_m':        round(float(centroid_world[2]), 3),
                })

                # Assign to nearest block if inside forest and the slot if in the arena ( Arena bool -> True )
                if (not self.in_arena):
                    block_num, dist = self.grid.nearest_block(centroid_world)   
                    if dist > self.cfg.ASSIGNMENT_MAX_DIST_MFF:
                        continue
                else:
                    # Column from lateral Y distance in robot frame
                    y_robot = float(centroid_robot[1])
                    if y_robot >= 0.8:
                        col = 0  # left
                    elif y_robot > 0.2 and y_robot < 0.8:
                        col = 1  # middle
                    elif y_robot <= 0.2 :
                        col = 2  # right

                    # Row from Z height — nearest of the three rack levels
                    # z = float(centroid_world[2])
                    # row_heights = [self.cfg.FIRST_ROW_Z, self.cfg.MID_ROW_Z, self.cfg.TOP_ROW_Z]
                    # row = int(np.argmin([abs(z - h) for h in row_heights]))
                    
                    z_robot = float(centroid_world[2])
                    if z_robot <= 0.8:
                        row = 0
                    elif z_robot < 1.5:
                        row = 1
                    else:
                        row = 2
                        
                        
                    slot_num = next(
                        s for s, (c, r) in self.rack.SLOT_LAYOUT.items()
                        if c == col and r == row
                    )
                    kfs_distances[-1]['slot_num'] = slot_num
                    kfs_distances[-1]['col'] = col
                    kfs_distances[-1]['row'] = row


                # Keep best-confidence match per block
                if (not self.in_arena):
                    existing = confirmed_grid[block_num]
                    if existing is not None and existing['confidence'] >= conf:
                        continue

                    confirmed_grid[block_num] = {
                        'kfs_type':   kfs_type,
                        'centroid':   centroid_robot.tolist(),
                        'confidence': round(conf, 3),
                        'distance_m': round(distance_m, 3),
                        'source':     'camera+lidar',
                    }
                else:
                    existing = confirmed_rack[slot_num]
                    if existing is not None and existing['confidence'] >= conf:
                        continue

                    confirmed_rack[slot_num] = {
                        'kfs_type':   whos_kfs,
                        'centroid':   centroid_robot.tolist(),
                        'confidence': round(conf, 3),
                        'distance_m': round(distance_m, 3),
                        'source':     'camera+lidar',
                    }
                

        # ── Publish ───────────────────────────────────────────────────────────
        active_dict = confirmed_rack if self.in_arena else confirmed_grid

        if self.in_arena:
            layout = self.rack.SLOT_LAYOUT
            msg = ArenaStatus()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            for slot_num, info in active_dict.items():
                if info is None:
                    continue
                col, row = layout[slot_num]
                slot_msg = KfsSlot()
                slot_msg.slot_num   = int(slot_num)
                slot_msg.col        = int(col)
                slot_msg.row        = int(row)
                slot_msg.kfs_type   = str(info['kfs_type'])
                slot_msg.confidence = float(info['confidence'])
                slot_msg.distance_m = float(info['distance_m'])
                msg.occupied_slots.append(slot_msg)
            self.arena_status.publish(msg)
        else:
            self.pub_confirmed.publish(String(data=json.dumps(active_dict, indent=2)))

        kfs_distances.sort(key=lambda d: d['distance_m'])
        self.pub_distances.publish(String(data=json.dumps(kfs_distances, indent=2)))

        # ── Debug image ───────────────────────────────────────────────────────
        if self.debug and image_msg is not None:
            try:
                debug_img = self._draw_debug(
                    image_msg, pixels, valid, active_dict, use_dets, kfs_distances,
                )
                self.pub_debug.publish(
                    self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                )
            except Exception as e:
                self.get_logger().warn(f"Debug image error: {e}")

        if self.in_arena:
            self.get_logger().debug(
                f"KFS confirmed (arena): {sum(1 for i in active_dict.values() if i)}"
            )
        else:
            r2_blocks = [b for b, i in active_dict.items() if i and i['kfs_type'] == 'R2_KFS']
            self.get_logger().debug(
                f"KFS confirmed: {sum(1 for i in active_dict.values() if i)}  |  "
                f"R2 targets: {r2_blocks}"
            )

    # ── Debug image ───────────────────────────────────────────────────────────

    def _draw_debug(self, image_msg, pixels, valid, confirmed, detections, kfs_distances=None):
        img = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        slot_prefix = 'S' if self.in_arena else 'B'

        # Camera bounding boxes — colour/label by auth class (Fake/R1/R2) + distance
        info_by_id = {str(d['track_id']): d for d in (kfs_distances or [])}
        if detections is not None:
            for det in detections.detections:
                type_id    = int(det.results[0].hypothesis.class_id) if det.results else -1
                team_id    = int(det.results[1].hypothesis.class_id) if len(det.results) > 1 else -1
                color      = CLASS_ID_TO_COLOR.get(type_id, (128, 128, 128))
                color_label = MAIN_CLASS_ID_TO_LABEL.get(team_id, '')
                cx = int(det.bbox.center.position.x)
                cy = int(det.bbox.center.position.y)
                hw = int(det.bbox.size_x / 2)
                hh = int(det.bbox.size_y / 2)
                
                cv2.rectangle(img, (cx - hw, cy - hh), (cx + hw, cy + hh), color, 2)
                type_label = CLASS_ID_TO_KFS_TYPE.get(type_id, '?')
                label      = f"[{color_label}] {type_label}" if color_label else type_label
                kfs_info   = info_by_id.get(str(det.id))
                
                if kfs_info is not None:
                    label += f"  {kfs_info['distance_m']:.2f}m"
                    if 'slot_num' in kfs_info:
                        label += f"  S{kfs_info['slot_num']} ({kfs_info['col']},{kfs_info['row']})"
                    cv2.putText(img, label, (cx - hw, cy - hh - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
                    cv2.putText(img, label, (cx - hw, cy - hh - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    xy_label = f"x:{kfs_info['x_m']:.2f}  y:{kfs_info['y_m']:.2f} z:{kfs_info['z_m']:.2f}"
                    cv2.putText(img, xy_label, (cx - hw, cy - hh - 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
                    cv2.putText(img, xy_label, (cx - hw, cy - hh - 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                    if 'col' in kfs_info:
                        idx_text = f"({kfs_info['col']},{kfs_info['row']})"
                        (tw, th), _ = cv2.getTextSize(idx_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
                        tx, ty = cx - tw // 2, cy + th // 2
                        cv2.putText(img, idx_text, (tx, ty),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
                        cv2.putText(img, idx_text, (tx, ty),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                else:
                    cv2.putText(img, label, (cx - hw, cy - hh - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
                    cv2.putText(img, label, (cx - hw, cy - hh - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # All projected LiDAR points — disabled
        # Confirmed KFS centroid circles/labels — disabled

        # Overlay text
        if not self.in_arena:
            r2_targets = [b for b, i in confirmed.items()
                          if i and i['kfs_type'] == 'R2_KFS']
            cv2.putText(img, f"R2 targets: {sorted(r2_targets)}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return img


# ─────────────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = LidarCameraFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
