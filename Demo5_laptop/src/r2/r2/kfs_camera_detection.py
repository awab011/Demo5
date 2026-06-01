"""
Author: Awab Ismail

Camera pipeline for detecting KFS

Pipeline:
    1. Capture frames in a dedicated thread, writing to a thread-safe buffer.
    2. Process frames in another thread:
        - Run main YOLO detection (Blue/Red KFS)
        - Format detections for tracker
        - Run StrongSORT tracker (with Re-ID every N frames)
        - For each track, run authenticity classifier and maintain vote history
        - Commit to a final class after enough votes, and prepare Detection2DArray message
    3. Publish Detection2DArray and debug image in a third thread at a fixed rate.

Subscribes: 
    - None (direct camera capture)

Publishes:
    - /kfs/detections (vision_msgs/Detection2DArray): Detected KFS with authenticity and type
    - /kfs/debug_image (sensor_msgs/Image): Annotated image for debugging
    - /camera1/image_raw (sensor_msgs/Image): Raw camera feed for monitoring

"""
import threading
import time
from pathlib import Path
from collections import defaultdict

import cv2
import numpy as np
import torch 
import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
from ultralytics import YOLO
from strongsort.strong_sort import StrongSORT
from datetime import datetime
# from TunedTracker import FastStrongSORT as StrongSORT

# Constants
CLASS_FAKE  = 0 
CLASS_R1    = 1
CLASS_R2    = 2
MAIN_MODEL_CLASS_NAMES = {
    0: "BLUE_KFS",
    1: "RED_KFS",
}
CLASS_NAMES = {
    CLASS_FAKE: 'Fake',
    CLASS_R1:   'R1',
    CLASS_R2:   'R2',}
VOTE_THRESHOLD       = 20
MIN_VOTES_TO_PUBLISH = 10
AUTH_FAIL_THRESHOLD  = 200

# Since threading is used, we need to ensure that the shared data structures are thread-safe.
class FrameBuffer:

    def __init__(self):
        self._frame = None
        self._lock  = threading.Lock()
        self._event = threading.Event()

    def write(self, frame):
        with self._lock:
            self._frame = frame
        self._event.set()

    def read(self, timeout =10):
        got = self._event.wait(timeout=timeout)
        if not got: 
            return None, False
        with self._lock:
            frame = self._frame.copy() if self._frame is not None else None
        self._event.clear()
        return frame, frame is not None

# Holds the lastest frame for publishing 
class ResultBuffer:

    def __init__(self):
        self._det_array   = None
        self._debug_frame = None
        self._lock        = threading.Lock()

    def write(self, det_array: Detection2DArray, debug_frame: np.ndarray):
        with self._lock:
            self._det_array   = det_array
            self._debug_frame = debug_frame

    def read(self):
        with self._lock:
            return self._det_array, self._debug_frame

class KFSCameraNode(Node):
    def __init__(self):
        super().__init__('kfs_camera_node')

        # Parameters
        self.model_path = self.declare_parameter(
            'main_model',   '/home/utmrbc/KFS-model/Main_9May.engine'). value 
        self.auth_model_path = self.declare_parameter(
            'auth_model',   '/home/utmrbc/weights_Demo2/ToBeUsed/Authenticity.engine'   ).value
        self.reid_weights = self.declare_parameter(
            'reid_weights', '/home/utmrbc/ros_yolo/reid_weights/osnet_x1_0_msmt17.pt'   ).value 
        self.camera_index    = self.declare_parameter('camera_index',   2).value
        self.img_width       = self.declare_parameter('img_width',   1280).value
        self.img_height      = self.declare_parameter('img_height',   720).value
        self.show            = self.declare_parameter('show'        ,True).value
        self.verbose         = self.declare_parameter('verbose',    False).value
        self.detect_conf     = self.declare_parameter('detect_conf',  0.8).value
        self.auth_conf       = self.declare_parameter('auth_conf',    0.2).value
        self.detect_class    = self.declare_parameter('detect_class',  [0,1]).value
        self.publish_hz      = self.declare_parameter('publish_hz',  100.0).value
        self.tracker_type    = self.declare_parameter('tracker_type', 'bytetrack').value  # 'bytetrack' or 'strongsort'

        # Publisher
        qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.pub_detections = self.create_publisher(Detection2DArray, '/kfs/detections', qos)
        self.pub_debug      = self.create_publisher(Image, '/kfs/debug_image', 10)
        self.pub_raw        = self.create_publisher(Image, '/camera1/image_raw', 10)

        # Buffers
        self.frame_buffer  = FrameBuffer()
        self.result_buffer = ResultBuffer()

        # Votes
        self.auth_votes     = defaultdict(list)
        self.auth_committed = {}
        self.auth_failed    = {}
        self.auth_failed_votes = {}
        self.auth_saved = {}

        self.bridge = CvBridge()
        self._running = True

        # Camera
        self.cap = cv2.VideoCapture('/dev/v4l/by-id/usb-Global_Shutter_Camera_Global_Shutter_Camera_01.00.00-video-index0',
                                     cv2.CAP_V4L2) 
        
        # self.cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
        self._setup_camera()
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to read from camera')
            self.cap.release()
            raise SystemExit("Camera initialization failed")
        self.get_logger().info('Camera initialized successfully')

        # Loading Models
        try: 
            self.model = YOLO(self.model_path, task='detect')
            self.auth_model = YOLO(self.auth_model_path, task='detect')
            self.get_logger().info('Models loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load models: {e}')
            self.cap.release()
            raise SystemExit("Model loading failed")

        # Tracker
        self.tracker = None
        if self.tracker_type == 'strongsort':
            for attempt in range(1, 4):
                try:
                    time.sleep(1.0)  # let TensorRT contexts settle before PyTorch claims the GPU
                    self.tracker = StrongSORT(
                        model_weights=Path(self.reid_weights),
                        device='cuda:0',
                        fp16=True,
                        max_dist=0.4,
                        max_iou_distance=0.7,
                        max_age=20,
                        n_init=5,
                        nn_budget=30,
                        mc_lambda=0.98,
                        ema_alpha=0.9,
                    )
                    self.get_logger().info(f"StrongSORT tracker created (attempt {attempt}).")
                    break
                except Exception as e:
                    self.get_logger().warn(f"Tracker init attempt {attempt}/3 failed: {e}")
                    if attempt == 3:
                        self.get_logger().error("StrongSORT failed after 3 attempts — tracker disabled.")
        else:
            self.get_logger().info("Using ByteTrack (built-in ultralytics tracker).")

        # Re-ID throttle: run full StrongSORT every N frames
        self._reid_every_n  = 3
        self._frame_count   = 0
        self._last_tracks   = np.empty((0, 8))

        # Per-step timing accumulators (averaged every N frames)
        self._t_yolo  = []
        self._t_sort  = []
        self._t_auth  = []
        self._t_ann   = []
        self._t_total = []
        self._avg_log_every = 30
        self._capture_fps   = 0.0

        # FPS counters
        self._cap_times = []
        self._proc_times = []

        # Threads
        self._capture_thread = threading.Thread(
            target=self._capture_loop, name='capture', daemon=True)
        self._process_thread = threading.Thread(
            target=self._process_loop, name='process', daemon=True)
        self._publish_thread = threading.Thread(
            target=self._publish_loop, name='publish', daemon=True)

        self._capture_thread.start()
        self._process_thread.start()
        self._publish_thread.start()

        self.get_logger().info(
            f"All threads started | publish_hz={self.publish_hz}"
        )

    def _capture_loop(self):

        counter = 0
        last_log = time.time()

        while rclpy.ok() and self._running:
            t0 = time.time()

            grabbed = self.cap.grab()
            if not grabbed:
                self.get_logger().error('Failed to grab frame')
                time.sleep(0.01)
                continue

            ret, frame = self.cap.retrieve()   
            if not ret:
                self.get_logger().error('Failed to retrieve frame')
                time.sleep(0.01)
                continue

            self.frame_buffer.write(frame)
            try:
                self.pub_raw.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
            except Exception:
                pass

            #FPS Tracking
            self._cap_times.append(time.time() - t0)
            if len(self._cap_times) > 100:
                self._cap_times.pop(0)
            counter += 1
            now = time.time()
            if now - last_log > 5:
                self._capture_fps = counter / (now - last_log)
                counter = 0
                last_log = now

    def _process_loop(self):

        last_log = time.time()
        while rclpy.ok() and self._running:
            try:
                t0 = time.time()

                frame, got = self.frame_buffer.read(timeout=1.0)
                if not got:
                    continue

                annotated, det_array = self._process_frame(frame)
                self.result_buffer.write(det_array, annotated)

                #FPS
                self._proc_times.append(time.time() - t0)
                if len(self._proc_times) > 100:
                    self._proc_times.pop(0)
                
                now = time.time()
                if now - last_log > 5.0:
                    proc_fps = 1.0 / (sum(self._proc_times) / len(self._proc_times))
                    self.get_logger().info(
                        f"[Process ] FPS: {proc_fps:.1f} | "
                        f"Committed: {len(self.auth_committed)} | "
                        f"Pending: {len(self.auth_votes) - len(self.auth_committed)}"
                    )
                    last_log = now

                
            except Exception as e:
                self.get_logger().error(f"Processing error: {e}")
                continue

    def _publish_loop(self):

        interval = 1.0 / self.publish_hz

        while rclpy.ok() and self._running:
            t0 = time.time()

            det_array, debug_frame = self.result_buffer.read()

            if det_array is not None:
                self.pub_detections.publish(det_array)

            if debug_frame is not None:
                try:
                    self.pub_debug.publish(
                        self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                    )
                except Exception as e:
                    self.get_logger().warn(f"Debug image publish failed: {e}")
                if self.show:
                    cv2.imshow('KFSCameraNode', debug_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self._running = False
                        # shutdown()
                        break
        # Sleep for remainder of interval
            elapsed = time.time() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _process_frame(self, frame: np.ndarray) -> tuple:

        
        det_array = Detection2DArray()
        det_array.header.stamp = self.get_clock().now().to_msg()
        det_array.header.frame_id = 'camera'

        tracks = np.empty((0, 8))
        t0 = time.time()

        if self.tracker_type == 'bytetrack':
            # ── ByteTrack: detection + tracking in one model.track() call ──
            try:
                results = self.model.track(
                    source=frame,
                    conf=self.detect_conf,
                    iou=0.25,
                    verbose=False,
                    classes=self.detect_class,
                    persist=True,
                    tracker='bytetrack.yaml',
                )
            except Exception as e:
                self.get_logger().warn(f"ByteTrack failed: {e}")
                return frame, det_array
            t1 = time.time()
            t2 = t1  # no separate format step
            if results[0].boxes is not None and results[0].boxes.id is not None:
                boxes = results[0].boxes
                xyxy  = boxes.xyxy.cpu().numpy()
                ids   = boxes.id.cpu().numpy().reshape(-1, 1)
                confs = boxes.conf.cpu().numpy().reshape(-1, 1)
                clss  = boxes.cls.cpu().numpy().reshape(-1, 1)
                pad   = np.zeros((len(ids), 1))
                tracks = np.hstack([xyxy, ids, confs, clss, pad])
            t3 = time.time()

        else:
            # ── StrongSORT: detect → format → track (Re-ID every N frames) ── 
            try:
                results = self.model.predict(
                    source=frame,
                    conf=self.detect_conf,
                    iou=0.25,
                    verbose=False,
                    classes=self.detect_class,
                )
            except Exception as e:
                self.get_logger().warn(f"Detection failed: {e}")
                return frame, det_array
            t1 = time.time()

            raw_dets = []
            if results[0].boxes is not None:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    cls  = int(box.cls[0])
                    raw_dets.append((x1, y1, x2, y2, conf, cls))
            dets_tensor = (
                torch.tensor(raw_dets, dtype=torch.float32) if raw_dets
                else torch.empty((0, 6), dtype=torch.float32)
            )
            t2 = time.time()

            self._frame_count += 1
            tracks = self._last_tracks
            if self.tracker is not None:
                try:
                    tracks = self.tracker.update(dets_tensor, frame)
                    self._last_tracks = tracks
                except Exception as e:
                    self.get_logger().warn(f"Tracker update failed: {e}")
            t3 = time.time()

        h, w = frame.shape[:2]
        auth_ms = 0.0
        ann_ms  = 0.0
        for track in tracks:
            # x1, y1, x2, y2, track_id, conf, cls, _ = track
            if len(track) >= 8:
                x1, y1, x2, y2, track_id, conf, cls, _ = track[:8]
            elif len(track) ==7:
                x1, y1, x2, y2, track_id, conf, cls = track[:7]
            else:
                continue

            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
            track_id = int(track_id)
            x1, y1   = max(0, x1), max(0, y1)
            x2, y2   = min(w, x2), min(h, y2)

            crop = frame[y1:y2, x1:x2]
            if crop.size == 0:
                continue

            ta = time.time()
            try:
                cls_id, cls_name, committed = self._classify(track_id, crop)
            except Exception as e:
                self.get_logger().warn(f"Classification failed for track: {e}")
                return 
            auth_ms += (time.time() - ta) * 1000

            if len(self.auth_votes[track_id]) >= MIN_VOTES_TO_PUBLISH:
                det      = Detection2D()
                det.header = det_array.header
                det.id   = str(track_id)

                bbox = BoundingBox2D()
                bbox.center.position.x = float((x1 + x2) / 2)
                bbox.center.position.y = float((y1 + y2) / 2)
                bbox.size_x = float(x2 - x1)
                bbox.size_y = float(y2 - y1)
                det.bbox = bbox

                # results[0] = auth model class (Fake / R1 / R2)
                hyp_auth = ObjectHypothesisWithPose()
                hyp_auth.hypothesis.class_id = str(cls_id)
                hyp_auth.hypothesis.score    = float(conf)
                det.results.append(hyp_auth)

                # results[1] = main detector class (Blue / Red)
                hyp_type = ObjectHypothesisWithPose()
                hyp_type.hypothesis.class_id = str(int(cls))
                hyp_type.hypothesis.score    = float(conf)
                det.results.append(hyp_type)

                det_array.detections.append(det)

            tb = time.time()
            # kfs_color = MAIN_MODEL_CLASS_NAMES[cls]
            color = self._class_color(cls_id, committed)
            status = 'COMMITTED' if committed else 'PENDING'
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
            cv2.putText(frame, f"{cls_name} {status} ID:{track_id}",
                        (x1, max(y1 - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
            ann_ms += (time.time() - tb) * 1000

        t4 = time.time()
        self._t_yolo.append((t1 - t0) * 1000)
        self._t_sort.append((t3 - t2) * 1000)
        self._t_auth.append(auth_ms)
        self._t_ann.append(ann_ms)
        self._t_total.append((t4 - t0) * 1000)

        if len(self._t_total) >= self._avg_log_every:
            def _avg(lst): return sum(lst) / len(lst)
            avg_total = _avg(self._t_total)
            self.get_logger().info(
                f"[{self.tracker_type.upper()}/AVG{self._avg_log_every}f] "
                f"YOLO:{_avg(self._t_yolo):.1f}ms "
                f"SORT:{_avg(self._t_sort):.1f}ms "
                f"AUTH:{_avg(self._t_auth):.1f}ms "
                f"ANN:{_avg(self._t_ann):.1f}ms "
                f"TOTAL:{avg_total:.1f}ms "
                f"PROC:{1000/avg_total:.1f}fps "
                f"CAP:{self._capture_fps:.1f}fps"
            )
            self._t_yolo.clear()
            self._t_sort.clear()
            self._t_auth.clear()
            self._t_ann.clear()
            self._t_total.clear()

        return frame, det_array

    def _classify(self, track_id: int, crop: np.ndarray) -> tuple:

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        if track_id in self.auth_committed:
            cls_id = self.auth_committed[track_id]
            if track_id not in self.auth_saved:
                gray_saved = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                cv2.imwrite(f"/home/utmrbc/auth_samples/track_{track_id}_class_{cls_id}_{timestamp}.jpg", gray_saved)
                self.auth_saved[track_id] = True
            return cls_id,   CLASS_NAMES[cls_id], True
        
        if track_id in self.auth_failed:
            cls_id = self.auth_failed[track_id]
            return -1, 'Not KFS', True

        if track_id not in self.auth_votes:
            self.auth_votes[track_id] = []
        if track_id not in self.auth_failed_votes:
            self.auth_failed_votes[track_id] = []

        try:
            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            gray_3ch = np.stack([gray] * 3, axis=-1)
            auth_res = self.auth_model.predict(
                source=gray_3ch,
                conf=self.auth_conf,
                device = 1,
                iou=0.25,
                verbose=False
            )
            if auth_res[0].boxes is not None and len(auth_res[0].boxes) > 0:
                best     = max(auth_res[0].boxes, key=lambda b: float(b.conf))
                voted    = int(best.cls)
                self.auth_votes[track_id].append(voted)
                if self.verbose:
                    self.get_logger().info(
                        f"Track {track_id}: vote {CLASS_NAMES[voted]} "
                        f"({len(self.auth_votes[track_id])}/{VOTE_THRESHOLD})"
                    )
            else:
                # voted_fail = "Failed"
                self.auth_failed_votes[track_id].append(1)

        except Exception as e:
            self.get_logger().warn(f"Authenticity classification failed for track {track_id}: {e}")

        votes = self.auth_votes[track_id]
        failed_votes = self.auth_failed_votes[track_id]

        # Commit if enough votes
        if len(votes) >= VOTE_THRESHOLD:
            cls_id = max(set(votes), key=votes.count)
            self.auth_committed[track_id] = cls_id
            self.get_logger().info(
                f"Track {track_id} → '{CLASS_NAMES[cls_id]}' committed "
                f"({votes.count(cls_id)}/{len(votes)} votes)"
            )
            return cls_id, CLASS_NAMES[cls_id], True

        if len(failed_votes) >= AUTH_FAIL_THRESHOLD:
            self.auth_failed[track_id] = "Not KFS"
            return -1, 'Not KFS', True

        # Uncommitted - return current best guess
        if votes: 
            best_guess = max(set(votes), key=votes.count)
            return best_guess, CLASS_NAMES[best_guess], False

        return -1 , 'Pending', False

    # HELPERS 

    def _class_color(self, cls_id: int, committed: bool) -> tuple:
        colors = {
            CLASS_R2:   (0, 255, 0),      # green  — collect
            CLASS_FAKE: (0, 0, 255),      # red    — avoid
            CLASS_R1:   (0, 165, 255),    # orange — ignore
            -1:         (128, 128, 128),  # gray   — pending
        }
        color = colors.get(cls_id, (128, 128, 128))
        return color if committed else tuple(int(c * 0.5) for c in color)

    def _setup_camera(self):
        self.cap.set(cv2.CAP_PROP_FOURCC,        cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,   self.img_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  self.img_height)
        self.cap.set(cv2.CAP_PROP_FPS,           90)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,    3)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS,    40)
        self.cap.set(cv2.CAP_PROP_CONTRAST,      0)
        self.cap.set(cv2.CAP_PROP_SATURATION,    80)
        self.cap.set(cv2.CAP_PROP_GAMMA,         110)
        self.cap.set(cv2.CAP_PROP_GAIN,          18)
        self.cap.set(cv2.CAP_PROP_AUTO_WB,       1.0)
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4200)
        self.cap.set(cv2.CAP_PROP_SHARPNESS,     7)
        self.cap.set(cv2.CAP_PROP_BACKLIGHT,     56)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE,      120)
        self.get_logger().info(
            f"Camera FPS: {self.cap.get(cv2.CAP_PROP_FPS)}"
        )

    def shutdown(self):
        if not self._running:
            return
        self._running = False

        # Join threads to ensure proper cleanup
        self.get_logger().info("Waiting for threads to finish...")
        self._capture_thread.join()
        self._process_thread.join()
        self._publish_thread.join()

        try:
            self.cap.release()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        self.get_logger().info("KFS camera node shut down.")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KFSCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as e:
        if node:
            node.get_logger().info(f"Exit: {e}")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()