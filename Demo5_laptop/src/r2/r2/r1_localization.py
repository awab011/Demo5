"""
Author: Awab Ismail
Date: 24/3/2026

R1 clustering and localization Node
===================================


ToDo List:
    1. Configs                      DONE
    2. R1 Detection                 DONE
    3. R1 localization              DONE
    4. Integration to aggregator    DONE
    5. Adding Filter                DONE
    6. Publishing markers           DONE

"""

import json
import struct
import time
from collections import deque

import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Vector3
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2


def pose_to_transform(pose: PoseStamped) -> np.ndarray:
    """Build a 4x4 robot-to-world transform from a PoseStamped message."""
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


class R1Config:

    R1_MAX_Z              = 1.0    # ignore points above this height

    # DBSCAN clustering
    R1_CLUSTER_EPS        = 0.2
    R1_CLUSTER_MIN_POINTS = 10

    # R1 approximate footprint size (meters)
    R1_MIN_XY_SIZE        = 0.8
    R1_MAX_XY_SIZE        = 1.1

    # Minimum points to consider detection high-confidence
    MIN_POINTS_CONFIDENT  = 50

    # Seconds before a cached position is considered stale
    MAX_POSITION_AGE      = 0.5

    # ── Tracker (StrongSORT-style state machine) ──────────────────────────────
    # Consecutive hits required before the track is published ("confirmed")
    TRACK_N_INIT          = 3
    # Consecutive missed frames before the track is dropped ("lost")
    TRACK_MAX_AGE         = 5


# ─────────────────────────────────────────────────────────────────────────────
#  R1 TRACKER  (StrongSORT-style state machine + Kalman filter)
# ─────────────────────────────────────────────────────────────────────────────

class R1Tracker:
    """
    Single-target tracker inspired by StrongSORT:

      TENTATIVE — cluster seen, but waiting for TRACK_N_INIT consecutive hits
                  before treating it as real (suppresses one-frame ghost hits).
      CONFIRMED — locked on. Position is published. Kalman prediction keeps the
                  track alive for up to TRACK_MAX_AGE missed frames (e.g. brief
                  occlusion) without needing a fresh detection every frame.
      LOST      — too many consecutive misses; track is cleared and must be
                  re-confirmed from scratch.

    State vector: [x, y, z, vx, vy, vz]  (constant-velocity model)
    """

    TENTATIVE = 'tentative'
    CONFIRMED = 'confirmed'
    LOST      = 'lost'

    def __init__(self, cfg: R1Config):
        self.cfg   = cfg
        self.state = self.TENTATIVE
        self.hits  = 0
        self.misses = 0
        self._init = False

        # Kalman state and covariance
        self._x = np.zeros(6)
        self._P = np.eye(6) * 10.0

        # State transition  [x y z vx vy vz] with dt=1 frame
        self._F = np.eye(6)
        self._F[0, 3] = 1.0
        self._F[1, 4] = 1.0
        self._F[2, 5] = 1.0

        # Measurement matrix  (we observe x, y, z only)
        self._H = np.zeros((3, 6))
        self._H[0, 0] = self._H[1, 1] = self._H[2, 2] = 1.0

        # Noise tuning
        self._Q = np.diag([0.05, 0.05, 0.05, 0.10, 0.10, 0.10])  # process
        self._R = np.diag([0.10, 0.10, 0.10])                     # measurement

    # ── public API ────────────────────────────────────────────────────────────

    @property
    def position(self) -> np.ndarray:
        return self._x[:3].copy()

    @property
    def is_confirmed(self) -> bool:
        return self.state == self.CONFIRMED

    @property
    def is_lost(self) -> bool:
        return self.state == self.LOST

    def update(self, measurement: np.ndarray):
        """Call when a valid R1 cluster is found this frame."""
        if not self._init:
            self._x[:3] = measurement
            self._init  = True
        else:
            self._predict_kf()
            self._update_kf(measurement)

        self.hits  += 1
        self.misses = 0
        if self.state == self.TENTATIVE and self.hits >= self.cfg.TRACK_N_INIT:
            self.state = self.CONFIRMED

    def miss(self):
        """Call when no matching cluster is found this frame."""
        if self._init:
            self._predict_kf()   # keep position estimate moving forward
        self.misses += 1
        if self.misses >= self.cfg.TRACK_MAX_AGE:
            self.state = self.LOST

    def reset(self):
        """Hard reset — called when track goes LOST."""
        self.__init__(self.cfg)

    # ── internal Kalman steps ─────────────────────────────────────────────────

    def _predict_kf(self):
        self._x = self._F @ self._x
        self._P = self._F @ self._P @ self._F.T + self._Q

    def _update_kf(self, z: np.ndarray):
        y = z - self._H @ self._x
        S = self._H @ self._P @ self._H.T + self._R
        K = self._P @ self._H.T @ np.linalg.inv(S)
        self._x = self._x + K @ y
        self._P = (np.eye(6) - K @ self._H) @ self._P


# ─────────────────────────────────────────────────────────────────────────────
#  R1 DETECTION
# ─────────────────────────────────────────────────────────────────────────────

def detect_r1(pcd: o3d.geometry.PointCloud, cfg: R1Config) -> dict:
    """
    Find R1 as the best-scoring cluster in the point cloud.

    Scoring weights size (closer to 0.6m = better) and point count.
    Returns dict:
        found       : bool
        centroid    : np.array [x, y, z] or None
        size_xy     : float
        points      : int
        confidence  : float 0-1
        all_clusters: list of dicts — every valid cluster (for debug visualisation)
    """
    _empty = {'found': False, 'centroid': None,
              'size_xy': 0.0, 'points': 0, 'confidence': 0.0,
              'all_clusters': []}

    if len(pcd.points) == 0:
        return _empty

    pts = np.asarray(pcd.points)

    # Height pre-filter
    pts = pts[pts[:, 2] <= cfg.R1_MAX_Z]
    if len(pts) < cfg.R1_CLUSTER_MIN_POINTS:
        return _empty

    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(pts)

    labels = np.array(filtered_pcd.cluster_dbscan(
        eps=cfg.R1_CLUSTER_EPS,
        min_points=cfg.R1_CLUSTER_MIN_POINTS,
        print_progress=False
    ))

    if labels.max() < 0:
        return _empty

    best_candidate = None
    best_score     = -1.0
    all_clusters   = []

    for label in set(labels):
        if label < 0:
            continue

        cluster_pts = pts[labels == label]
        n           = len(cluster_pts)

        xy_min  = cluster_pts[:, :2].min(axis=0)
        xy_max  = cluster_pts[:, :2].max(axis=0)
        xy_size = float(max(xy_max - xy_min))

        if xy_size < cfg.R1_MIN_XY_SIZE or xy_size > cfg.R1_MAX_XY_SIZE:
            # Still record it for debug, just mark as rejected
            all_clusters.append({
                'points':   cluster_pts,
                'centroid': cluster_pts.mean(axis=0),
                'size_xy':  xy_size,
                'n':        n,
                'score':    0.0,
                'rejected': True,
            })
            continue

        size_score  = 1.0 - abs(xy_size - 0.6) / 0.6
        point_score = min(n / 200.0, 1.0)
        score       = size_score * 0.6 + point_score * 0.4

        all_clusters.append({
            'points':   cluster_pts,
            'centroid': cluster_pts.mean(axis=0),
            'size_xy':  xy_size,
            'n':        n,
            'score':    round(score, 3),
            'rejected': False,
        })

        if score > best_score:
            best_score     = score
            best_candidate = {
                'found':      True,
                'centroid':   cluster_pts.mean(axis=0),
                'size_xy':    xy_size,
                'points':     n,
                'confidence': round(score, 3),
                'all_clusters': all_clusters,
            }

    if best_candidate is None:
        return {**_empty, 'all_clusters': all_clusters}

    best_candidate['all_clusters'] = all_clusters
    return best_candidate



class R1LocalizationNode(Node):

    def __init__(self):
        super().__init__('r1_localization_node')

        self.cfg             = R1Config()
        self.tracker         = R1Tracker(self.cfg)
        self.last_seen       = None
        self.latest_pose     = None
        self._zone_x_history = deque(maxlen=10)

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(
            PointCloud2,
            '/r2/lidar/preprocessed',
            self.cloud_callback,
            qos_sensor
        )
        self.create_subscription(
            PoseStamped,
            '/r2/pose',
            self._pose_cb,
            qos_sensor
        )

        self.pub_pose          = self.create_publisher(PoseStamped, '/r1/position',       10)
        self.pub_status        = self.create_publisher(String,      '/r1/status',         10)
        self.pub_debug_cloud   = self.create_publisher(PointCloud2, '/r1/debug/clusters', 10)
        self.pub_debug_markers = self.create_publisher(MarkerArray, '/r1/debug/markers',  10)

        self.get_logger().info(
            f"R1LocalizationNode ready | "
            f"eps={self.cfg.R1_CLUSTER_EPS}m  "
            f"size={self.cfg.R1_MIN_XY_SIZE}–{self.cfg.R1_MAX_XY_SIZE}m  "
            f"n_init={self.cfg.TRACK_N_INIT}  max_age={self.cfg.TRACK_MAX_AGE}"
        )

    def cloud_callback(self, msg: PointCloud2):
        pts = [
            [p[0], p[1], p[2]]
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        ]
        if not pts:
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(pts, dtype=np.float32))

        result = detect_r1(pcd, self.cfg)
        now    = time.time()

        if result['found']:
            self.tracker.update(result['centroid'])
            self.last_seen = now
            pos = self.tracker.position
            self.get_logger().debug(
                f"R1 {self.tracker.state} | "
                f"hits={self.tracker.hits}  misses={self.tracker.misses}  "
                f"pos=({pos[0]:.2f}, {pos[1]:.2f})  "
                f"size={result['size_xy']:.2f}m  pts={result['points']}  "
                f"conf={result['confidence']:.2f}"
            )
        else:
            self.tracker.miss()
            if self.tracker.is_lost:
                self.get_logger().info("R1 track LOST — waiting for re-confirmation")
                self.tracker.reset()
                return

        # Only publish once confirmed
        if not self.tracker.is_confirmed:
            self._publish_debug(result['all_clusters'], result['centroid'], msg.header.frame_id)
            return

        # Staleness check (wall-clock, catches GLIM dropout)
        age = now - self.last_seen if self.last_seen else float('inf')
        if age > self.cfg.MAX_POSITION_AGE:
            self.get_logger().warn(f"R1 position stale ({age:.1f}s)", throttle_duration_sec=2.0)

        self._publish(result['found'], result['confidence'])
        self._publish_debug(result['all_clusters'], result['centroid'], msg.header.frame_id)

    def _pose_cb(self, msg: PoseStamped):
        self.latest_pose = msg

    def _publish(self, detected: bool, confidence: float):
        stamp = self.get_clock().now().to_msg()

        # Transform R1 centroid from robot frame → map frame using R2 pose
        rx, ry, rz = self.tracker.position
        if self.latest_pose is not None:
            robot_T  = pose_to_transform(self.latest_pose)
            world_pt = robot_T @ np.array([rx, ry, rz, 1.0])
            wx, wy, wz = world_pt[0], world_pt[1], world_pt[2]
        else:
            wx, wy, wz = rx, ry, rz
            self.get_logger().warn("No /r2/pose received yet — R1 position in robot frame")

        pose_msg = PoseStamped()
        pose_msg.header.stamp    = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(wx)
        pose_msg.pose.position.y = float(wy)
        pose_msg.pose.position.z = float(wz)
        pose_msg.pose.orientation.w = 1.0
        self.pub_pose.publish(pose_msg)

        status = {
            'x':          round(float(wx), 3),
            'y':          round(float(wy), 3),
            'z':          round(float(wz), 3),
            'detected':   detected,
            'confidence': confidence,
            'zone':       self._position_to_zone(wx, wy),
        }
        self.pub_status.publish(String(data=json.dumps(status)))

    def _publish_debug(self, all_clusters: list, r1_centroid, frame_id: str):
        """
        Publish two debug topics for RViz2:
          /r1/debug/clusters  — PointCloud2 with each cluster in a different colour
          /r1/debug/markers   — MarkerArray with centroid spheres + text labels
        """
        # Colour palette — one colour per cluster index
        PALETTE = [
            (0.2, 0.6, 1.0),   # blue
            (1.0, 0.8, 0.0),   # yellow
            (1.0, 0.4, 0.8),   # pink
            (0.6, 0.2, 1.0),   # purple
            (0.0, 1.0, 1.0),   # cyan
            (1.0, 0.5, 0.0),   # orange
            (0.8, 0.8, 0.8),   # gray
        ]
        R1_COLOR   = (0.2, 1.0, 0.4)   # green  — confirmed R1
        REJECT_COL = (0.35, 0.35, 0.35) # dark gray      — rejected (wrong size)

        stamp    = self.get_clock().now().to_msg()
        header   = Header(stamp=stamp, frame_id=frame_id)

        # ── Coloured PointCloud2 ──────────────────────────────────────────────
        fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = []
        marker_array = MarkerArray()

        for i, cl in enumerate(all_clusters):
            if cl['rejected']:
                r, g, b = REJECT_COL
            elif r1_centroid is not None and np.allclose(cl['centroid'], r1_centroid, atol=0.01):
                r, g, b = R1_COLOR
            else:
                r, g, b = PALETTE[i % len(PALETTE)]

            ri, gi, bi = int(r * 255), int(g * 255), int(b * 255)
            rgb_packed  = struct.unpack('f',
                struct.pack('I', (ri << 16) | (gi << 8) | bi)
            )[0]

            for pt in cl['points']:
                cloud_data.append([float(pt[0]), float(pt[1]), float(pt[2]), rgb_packed])

            # ── Sphere marker at centroid ────────────────────────────────────
            cx, cy, cz = cl['centroid']
            sphere = Marker()
            sphere.header        = header
            sphere.ns            = 'r1_clusters'
            sphere.id            = i * 2
            sphere.type          = Marker.SPHERE
            sphere.action        = Marker.ADD
            sphere.pose.position = Point(x=float(cx), y=float(cy), z=float(cz))
            sphere.pose.orientation.w = 1.0
            sphere.scale         = Vector3(x=cl['size_xy'], y=cl['size_xy'], z=0.1)
            sphere.color         = ColorRGBA(r=r, g=g, b=b, a=0.45)
            sphere.lifetime.sec  = 1
            marker_array.markers.append(sphere)

            # ── Text label ───────────────────────────────────────────────────
            label_str = (
                f"{'R1' if (r1_centroid is not None and np.allclose(cl['centroid'], r1_centroid, atol=0.01)) else ('X' if cl['rejected'] else str(i))}"
                f"  {cl['size_xy']:.2f}m  n={cl['n']}"
                + (f"  s={cl['score']:.2f}" if not cl['rejected'] else "  [rejected]")
            )
            text = Marker()
            text.header        = header
            text.ns            = 'r1_labels'
            text.id            = i * 2 + 1
            text.type          = Marker.TEXT_VIEW_FACING
            text.action        = Marker.ADD
            text.pose.position = Point(x=float(cx), y=float(cy), z=float(cz) + 0.25)
            text.pose.orientation.w = 1.0
            text.scale.z       = 0.12
            text.color         = ColorRGBA(r=r, g=g, b=b, a=1.0)
            text.text          = label_str
            text.lifetime.sec  = 1
            marker_array.markers.append(text)

        if cloud_data:
            cloud_msg = pc2.create_cloud(header, fields, cloud_data)
            self.pub_debug_cloud.publish(cloud_msg)

        self.pub_debug_markers.publish(marker_array)

    def _position_to_zone(self, x: float, y: float) -> str:
        """
        Classify R1 position into field zones based on X coordinate.
        Boundaries need to be tuned to match the actual field layout.
        """
        self._zone_x_history.append(x)
        smoothed_x = sum(self._zone_x_history) / len(self._zone_x_history)

        if smoothed_x < 1.6:
            return 'Martial_club'
        elif smoothed_x < 2.2:
            return 'Forest_entrance'
        elif smoothed_x < 8.0:
            return 'Forest'
        elif smoothed_x < 12.0:
            return 'Arena'
        else:
            return 'Unknown'



def main(args=None):
    rclpy.init(args=args)
    node = R1LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
