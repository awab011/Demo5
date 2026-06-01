"""
Author: Awab Ismail
Date : 19-03-2026

Aggregator node — sums up data from different ROS nodes, packs into classic
CAN frames (8 bytes max), and sends them to the STM32 mainboard with
priority-aware scheduling.
--------------------------------------------------------------------
CAN ID allocation (laptop -> H7), classic CAN @ 1 Mbps:

  0x100  R2_PATH       — r2can multi-frame path packet (bursty)
  0x101  CRITICAL_CMD  — r2_h_cmd (incl. E_STOP) + target_x/y/yaw + seq
                          ON CHANGE (urgent) + 20 Hz periodic
  0x102  ZONES         — r2_zone + r1_zone + required_navi_state + seq
                          ON CHANGE (urgent) + 10 Hz periodic
  0x103  R2_POSE       — x/y/z_mm + yaw_cd                     (20 Hz)
  0x104  R1_POSE       — r1_x/y_mm                              (5 Hz)

  0x110  H7_FEEDBACK   — R2Outgoing_t  (H7 -> laptop, unchanged)

Lower CAN IDs win bus arbitration → 0x101 always beats 0x103/0x104.
On top of bus arbitration, the sender keeps a software priority queue so
urgent frames (E_STOP, new target) jump ahead of any periodic backlog.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from r2_msgs.msg import CustomMsg
from geometry_msgs.msg import PoseStamped
from r2_navigation.msg import R2Path
import json
import math
import time
import struct
import threading


KFS_TO_INT = {
    None:               0,
    'R1_KFS':           1,
    'R2_KFS':           2,
    'FAKE_KFS':         9,
    'UNKNOWN':          0,
    'R1_KFS_PROBABLE':  1,
    'R2_KFS_PROBABLE':  2,
}

# Maps zone string to Zone_t enum: MARTIAL_CLUB=0, FOREST=1, ARENA=2
ZONE_TO_INT = {
    'Martial_club':     0,
    'Forest_entrance':  1,
    'Forest':           1,
    'Arena':            2,
    'Unknown':          0,
    '':                 0,
}

# Maps command string to R2_Cmd_t enum: GO=0, PICK=1, CONTINUE=2, E_STOP=3
CMD_TO_INT = {
    'GO':       0, 'go':       0,
    'PICK':     1, 'pick':     1,
    'CONTINUE': 2, 'continue': 2,
    'E_STOP':   3, 'e_stop':   3, 'ESTOP': 3,
}

# CAN IDs (laptop -> H7)
CAN_ID_R2_PATH        = 0x100
CAN_ID_CRITICAL_CMD   = 0x101
CAN_ID_ZONES          = 0x102
CAN_ID_R2_POSE        = 0x103
CAN_ID_R1_POSE        = 0x104

# CAN ID (H7 -> laptop)
CAN_ID_H7_FEEDBACK    = 0x110

# Frame periods (seconds)
PERIOD_CRITICAL_CMD   = 0.05    # 20 Hz heartbeat (also pushed on change)
PERIOD_ZONES          = 0.10    # 10 Hz heartbeat (also pushed on change)
PERIOD_R2_POSE        = 0.05    # 20 Hz pose stream
PERIOD_R1_POSE        = 0.20    # 5 Hz

# R2Outgoing_t: encoder_x(u32) encoder_y(u32) r2_yaw(u16) navi_state(u8) completed(u8)
_R2OUT_FMT  = '<IIHBB'
_R2OUT_SIZE = struct.calcsize(_R2OUT_FMT)  # 12 bytes

_R2CAN_MSG_ID          = 0xA2
_R2CAN_MAX_STEPS       = 16
_R2CAN_STEPS_PER_FRAME = 2
_R2CAN_HDELTA_ZERO     = 2

_BLOCK_HEIGHT_MM = [0, 400, 200, 400, 200, 400, 600, 400, 600, 400, 200, 400, 200]


# ── r2can helpers (unchanged from previous protocol) ─────────────────────────

def _r2can_crc8(buf):
    crc = 0
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def _r2can_pack_grid(grid_state):
    out = [0, 0, 0]
    for i in range(12):
        byte_i = (i * 2) // 8
        bit_i  = (i * 2) % 8
        out[byte_i] |= (grid_state[i] & 0x03) << bit_i
    return out


def _r2can_encode_step(s):
    return (
        ((s.action_type            & 0x03) << 14) |
        ((s.current_block          & 0x0F) << 10) |
        (((s.target_block - 1)     & 0x0F) <<  6) |
        ((s.direction              & 0x03) <<  4) |
        ((s.collected_after        & 0x03) <<  2) |
        ((1 if s.requires_r1_clear else 0) <<  1) |
        ((1 if s.auto_pickup       else 0)       )
    ) & 0xFFFF


def _r2can_height_enc(cur, tgt):
    if cur < 1 or cur > 12 or tgt < 1 or tgt > 12:
        return _R2CAN_HDELTA_ZERO
    d = _BLOCK_HEIGHT_MM[tgt] - _BLOCK_HEIGHT_MM[cur]
    return {-400: 0, -200: 1, 0: 2, 200: 3, 400: 4}.get(d, _R2CAN_HDELTA_ZERO)


def _r2can_pre_entry_height_enc(tgt):
    if tgt < 1 or tgt > 12:
        return _R2CAN_HDELTA_ZERO
    return {200: 3, 400: 4, 600: 5}.get(_BLOCK_HEIGHT_MM[tgt], _R2CAN_HDELTA_ZERO)


def r2can_pack(msg):
    """Convert an R2Path ROS msg to a list of 8-byte CAN frames (bytes objects)."""
    step_count   = min(len(msg.steps), _R2CAN_MAX_STEPS)
    step_frames  = (step_count + _R2CAN_STEPS_PER_FRAME - 1) // _R2CAN_STEPS_PER_FRAME if step_count else 0
    total_frames = 1 + step_frames

    frames = []

    f0 = bytearray(8)
    f0[0] = _R2CAN_MSG_ID
    f0[1] = total_frames
    f0[2] = msg.total_cost & 0xFF
    entry_enc = (msg.entry_block - 1) & 0x03
    exit_enc  = (msg.exit_block - 10) & 0x03
    pre_enc   = len(msg.pre_entry_pickups) & 0x07
    f0[3] = ((exit_enc << 6) | (entry_enc << 4) | (pre_enc << 1)) & 0xFF
    gp = _r2can_pack_grid(list(msg.grid_state))
    f0[4], f0[5], f0[6] = gp
    f0[7] = _r2can_crc8(f0[:7])
    frames.append(bytes(f0))

    si = 0
    for fi in range(1, total_frames):
        fn = bytearray(8)
        fn[0] = fi
        steps_here = 0
        h0 = h1 = _R2CAN_HDELTA_ZERO
        for s_idx in range(_R2CAN_STEPS_PER_FRAME):
            if si >= step_count:
                break
            sp = msg.steps[si]; si += 1
            w = _r2can_encode_step(sp)
            fn[2 + s_idx * 2] = (w >> 8) & 0xFF
            fn[3 + s_idx * 2] = w & 0xFF
            if sp.current_block == 0:
                enc = _r2can_pre_entry_height_enc(sp.target_block)
            else:
                enc = (sp.height_delta_enc
                       if (sp.height_delta_enc != _R2CAN_HDELTA_ZERO or sp.action_type == 0)
                       else _r2can_height_enc(sp.current_block, sp.target_block))
            if s_idx == 0:
                h0 = enc
            else:
                h1 = enc
            steps_here += 1
        fn[1] = steps_here
        fn[6] = (((h0 & 0x07) << 5) | ((h1 & 0x07) << 2)) & 0xFF
        fn[7] = _r2can_crc8(fn[:7])
        frames.append(bytes(fn))

    return frames


# ── Classic-CAN frame packers (each returns exactly 8 bytes) ────────────────

def _clamp16(v):
    return max(-32768, min(32767, int(v)))


def pack_critical_cmd(r2_h_cmd, target_x_mm, target_y_mm, target_yaw_cd, seq):
    """0x101: cmd(1) + tx(2) + ty(2) + tyaw(2) + seq(1) = 8 B."""
    return struct.pack('<BhhhB',
                       r2_h_cmd & 0xFF,
                       target_x_mm, target_y_mm, target_yaw_cd,
                       seq & 0xFF)


def pack_zones(r2_zone, r1_zone, required_navi_state, seq):
    """0x102: r2_zone(1) + r1_zone(1) + navi(1) + seq(1) + reserved(4) = 8 B."""
    return struct.pack('<BBBB4x',
                       r2_zone & 0xFF, r1_zone & 0xFF,
                       required_navi_state & 0xFF, seq & 0xFF)


def pack_r2_pose(x_mm, y_mm, z_mm, yaw_cd):
    """0x103: 4 × int16 = 8 B."""
    return struct.pack('<hhhh', x_mm, y_mm, z_mm, yaw_cd)


def pack_r1_pose(r1_x_mm, r1_y_mm):
    """0x104: 2 × int16 + reserved(4) = 8 B."""
    return struct.pack('<hh4x', r1_x_mm, r1_y_mm)


# ── CAN sender / receiver (classic CAN, priority queue) ──────────────────────

class CANSender:
    URGENT = 0   # lower number = higher priority in PriorityQueue
    NORMAL = 1

    def __init__(self, channel='can0', bustype='socketcan', bitrate=1000000,
                 rx_callback=None):
        self.enabled = False
        self.bus     = None
        self._last_err_time = 0.0
        self._queue       = None
        self._counter     = 0
        self._counter_lock = threading.Lock()
        self._tx_thread   = None
        self._rx_thread   = None
        self._rx_callback = rx_callback

        try:
            import can
            import queue
            self.bus = can.interface.Bus(channel=channel, bustype=bustype,
                                         bitrate=bitrate)
            self.enabled = True
            # (priority, counter, frames_or_None_for_shutdown)
            self._queue = queue.PriorityQueue(maxsize=256)
            self._tx_thread = threading.Thread(target=self._worker, daemon=True)
            self._tx_thread.start()
            if rx_callback is not None:
                self._rx_thread = threading.Thread(target=self._receiver, daemon=True)
                self._rx_thread.start()
            print(f"CAN initialized on {channel} @ {bitrate} bps (classic CAN)")
        except Exception as e:
            print(f"CAN init failed: {e}")

    def _next_counter(self):
        with self._counter_lock:
            self._counter += 1
            return self._counter

    def send_frames(self, frames, urgent=False):
        """Enqueue an iterable of (can_id, data_bytes) tuples for transmission."""
        if not self.enabled or self._queue is None:
            return
        pri = self.URGENT if urgent else self.NORMAL
        try:
            self._queue.put_nowait((pri, self._next_counter(), frames))
        except Exception:
            pass  # queue full — drop

    def _worker(self):
        import can
        import queue as _q
        while True:
            try:
                pri, _, frames = self._queue.get(timeout=1.0)
            except _q.Empty:
                continue
            if frames is None:   # shutdown signal
                break
            mark = '!' if pri == self.URGENT else ' '
            for can_id, data in frames:
                msg = can.Message(
                    arbitration_id=can_id,
                    data=data,
                    is_extended_id=False,
                )
                t_send = time.monotonic()
                try:
                    self.bus.send(msg, timeout=0.01)
                    dt_ms = (time.monotonic() - t_send) * 1000
                    print(f"[CAN TX{mark}] 0x{can_id:03X} {len(data)}B ACK {dt_ms:.2f} ms")
                except Exception as e:
                    now = time.monotonic()
                    if now - self._last_err_time > 5.0:
                        print(f"[CAN TX] 0x{can_id:03X} FAILED: {e}")
                        self._last_err_time = now
                time.sleep(0.0005)

    def _receiver(self):
        while True:
            try:
                msg = self.bus.recv(timeout=1.0)
                if msg is None:
                    continue
                if self._rx_callback:
                    self._rx_callback(msg.arbitration_id, bytes(msg.data))
            except Exception:
                pass

    def close(self):
        if self._queue is not None:
            try:
                self._queue.put_nowait((self.NORMAL, self._next_counter(), None))
            except Exception:
                pass
        if self.bus:
            self.bus.shutdown()


# ── Aggregator node ──────────────────────────────────────────────────────────

class AggregatorNode(Node):
    def __init__(self):
        super().__init__('aggregator_node')

        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_bitrate', 1000000)

        can_channel = self.get_parameter('can_channel').value
        can_bitrate = self.get_parameter('can_bitrate').value

        self.can_sender = CANSender(channel=can_channel, bitrate=can_bitrate,
                                    rx_callback=self._on_h7_can)

        self.lock = threading.Lock()

        # R2 pose (from /r2/pose)
        self.x = 0.0; self.y = 0.0; self.z = 0.0; self.yaw = 0.0
        # Forest state (from /r2/forest/confirmed_grid)
        self.forest        = [0] * 12
        self.current_block = 0
        # R1 status (from /r1/status)
        self.r1_x = 0.0; self.r1_y = 0.0
        self.r1_position = ''
        self.r1_command  = ''
        self.r1_zone     = 0
        # High-level command (from /r2/cmd/high_level)
        self.r2_zone             = 0
        self.r2_h_cmd            = 0
        self.required_navi_state = 0
        self.target_x            = 0.0
        self.target_y            = 0.0
        self.target_yaw          = 0.0
        # Status flags
        self.lidar_ok  = False
        self.camera_ok = False
        self.end       = False

        # Per-frame sequence counters
        self._seq_cmd   = 0
        self._seq_zones = 0

        # Publishers
        self.publisher_   = self.create_publisher(CustomMsg, 'R2_message', 10)
        self.h7_state_pub = self.create_publisher(String, '/r2/h7_state', 10)

        # Subscribers
        self.create_subscription(String,      '/r2/forest/confirmed_grid',
                                 self.vision_callback, 10)
        self.create_subscription(PoseStamped, '/r2/pose',
                                 self.localization_callback, 10)
        self.create_subscription(String,      '/r1/status',
                                 self.r1_status_callback, 10)
        self.create_subscription(R2Path,      '/r2_path',
                                 self.r2_path_callback, 10)
        self.create_subscription(String,      '/r2/cmd/high_level',
                                 self.high_level_cmd_callback, 10)

        # Periodic timers — one per frame, each at its own rate.
        self.create_timer(PERIOD_CRITICAL_CMD, self._tick_critical_cmd)
        self.create_timer(PERIOD_ZONES,        self._tick_zones)
        self.create_timer(PERIOD_R2_POSE,      self._tick_r2_pose)
        self.create_timer(PERIOD_R1_POSE,      self._tick_r1_pose)
        # Legacy CustomMsg ROS publisher heartbeat
        self.create_timer(0.1, self._publish_custom_msg_tick)

        self.get_logger().info(
            f"Aggregator Node started | CAN: {self.can_sender.enabled} (classic, 1 Mbps)"
        )

    # ── Subscription callbacks ───────────────────────────────────────────────

    def vision_callback(self, msg):
        try:
            grid = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"vision JSON: {e}")
            return
        forest = [0] * 12
        for block_str, info in grid.items():
            try:
                block_num = int(block_str)
            except (TypeError, ValueError):
                continue
            if block_num < 1 or block_num > 12:
                continue
            idx = block_num - 1
            if info is None:
                forest[idx] = 0
            else:
                kfs_type = info.get('kfs_type', 'UNKNOWN')
                forest[idx] = KFS_TO_INT.get(kfs_type, 0)
        with self.lock:
            self.forest    = forest
            self.camera_ok = True
        self.get_logger().info(f"forest: {forest}")

    def localization_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        current_block = self._position_to_block(x, y)
        with self.lock:
            self.x, self.y, self.z, self.yaw = x, y, z, yaw
            self.current_block = current_block
            self.lidar_ok = True

    def r1_status_callback(self, msg):
        try:
            data        = json.loads(msg.data)
            r1_x        = float(data.get('x', 0.0))
            r1_y        = float(data.get('y', 0.0))
            r1_position = str(data.get('zone', ''))
            r1_command  = str(data.get('command', ''))
        except json.JSONDecodeError as e:
            self.get_logger().error(f"R1 status JSON: {e}")
            return
        new_zone = ZONE_TO_INT.get(r1_position, 0)
        with self.lock:
            zone_changed = (new_zone != self.r1_zone)
            self.r1_x, self.r1_y = r1_x, r1_y
            self.r1_position = r1_position
            self.r1_command  = r1_command
            self.r1_zone     = new_zone
        # r1_zone is part of the ZONES frame — push immediately on change.
        if zone_changed:
            self._send_zones(urgent=True)

    def r2_path_callback(self, msg):
        try:
            frames = r2can_pack(msg)
        except Exception as e:
            self.get_logger().error(f"r2can_pack: {e}")
            return
        self.get_logger().info(
            f"[R2Path] entry=B{msg.entry_block} exit=B{msg.exit_block} "
            f"cost={msg.total_cost} steps={len(msg.steps)} -> {len(frames)} frame(s)"
        )
        # Path packet is at 0x100 (highest hardware-arbitration priority); it
        # plays at normal software-priority so it doesn't fight E_STOP bursts.
        self.can_sender.send_frames([(CAN_ID_R2_PATH, f) for f in frames])

    def high_level_cmd_callback(self, msg):
        """Parse JSON command from r2_brain and push the relevant frame(s)."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"high_level_cmd JSON: {e}")
            return

        r2_zone_raw = data.get('r2_zone', 0)
        new_r2_zone = (r2_zone_raw if isinstance(r2_zone_raw, int)
                       else ZONE_TO_INT.get(str(r2_zone_raw), 0))

        r2_cmd_raw  = data.get('r2_h_cmd', 0)
        new_r2_cmd  = (r2_cmd_raw if isinstance(r2_cmd_raw, int)
                       else CMD_TO_INT.get(str(r2_cmd_raw), 0))

        new_navi  = int(data.get('required_navi_state', 0))
        new_tx    = float(data.get('target_x', 0.0))
        new_ty    = float(data.get('target_y', 0.0))
        new_tyaw  = float(data.get('target_yaw', 0.0))

        with self.lock:
            # ---- CRITICAL_CMD frame: r2_h_cmd + target_{x,y,yaw} ----
            critical_changed = (
                new_r2_cmd != self.r2_h_cmd or
                new_tx     != self.target_x or
                new_ty     != self.target_y or
                new_tyaw   != self.target_yaw
            )
            # ---- ZONES frame: r2_zone + required_navi_state ----
            zones_changed = (
                new_r2_zone != self.r2_zone or
                new_navi    != self.required_navi_state
            )

            self.r2_zone             = new_r2_zone
            self.r2_h_cmd            = new_r2_cmd
            self.required_navi_state = new_navi
            self.target_x            = new_tx
            self.target_y            = new_ty
            self.target_yaw          = new_tyaw

        # ON-CHANGE push: jump the queue with URGENT priority.
        # E_STOP / new target reach the H7 within ~1 ms regardless of backlog.
        if critical_changed:
            self.get_logger().info(
                f"[URGENT] cmd={new_r2_cmd} "
                f"target=({new_tx:.2f},{new_ty:.2f},{math.degrees(new_tyaw):.1f}deg)"
            )
            self._send_critical_cmd(urgent=True)
        if zones_changed:
            self._send_zones(urgent=True)

    # ── Frame builders ───────────────────────────────────────────────────────

    def _build_critical_cmd_payload(self):
        with self.lock:
            cmd     = self.r2_h_cmd
            tx_mm   = _clamp16(self.target_x * 1000)
            ty_mm   = _clamp16(self.target_y * 1000)
            tyaw_cd = _clamp16(math.degrees(self.target_yaw) * 100)
        self._seq_cmd = (self._seq_cmd + 1) & 0xFF
        return pack_critical_cmd(cmd, tx_mm, ty_mm, tyaw_cd, self._seq_cmd)

    def _build_zones_payload(self):
        with self.lock:
            r2z  = self.r2_zone
            r1z  = self.r1_zone
            navi = self.required_navi_state
        self._seq_zones = (self._seq_zones + 1) & 0xFF
        return pack_zones(r2z, r1z, navi, self._seq_zones)

    def _build_r2_pose_payload(self):
        with self.lock:
            x_mm   = _clamp16(self.x * 1000)
            y_mm   = _clamp16(self.y * 1000)
            z_mm   = _clamp16(self.z * 1000)
            yaw_cd = _clamp16(math.degrees(self.yaw) * 100)
        return pack_r2_pose(x_mm, y_mm, z_mm, yaw_cd)

    def _build_r1_pose_payload(self):
        with self.lock:
            r1_x_mm = _clamp16(self.r1_x * 1000)
            r1_y_mm = _clamp16(self.r1_y * 1000)
        return pack_r1_pose(r1_x_mm, r1_y_mm)

    # ── Per-frame senders (urgent vs normal) ─────────────────────────────────

    def _send_critical_cmd(self, urgent):
        self.can_sender.send_frames(
            [(CAN_ID_CRITICAL_CMD, self._build_critical_cmd_payload())],
            urgent=urgent,
        )

    def _send_zones(self, urgent):
        self.can_sender.send_frames(
            [(CAN_ID_ZONES, self._build_zones_payload())],
            urgent=urgent,
        )

    def _send_r2_pose(self, urgent):
        self.can_sender.send_frames(
            [(CAN_ID_R2_POSE, self._build_r2_pose_payload())],
            urgent=urgent,
        )

    def _send_r1_pose(self, urgent):
        self.can_sender.send_frames(
            [(CAN_ID_R1_POSE, self._build_r1_pose_payload())],
            urgent=urgent,
        )

    # ── Periodic timer callbacks ─────────────────────────────────────────────

    def _tick_critical_cmd(self): self._send_critical_cmd(urgent=False)
    def _tick_zones(self):        self._send_zones(urgent=False)
    def _tick_r2_pose(self):      self._send_r2_pose(urgent=False)
    def _tick_r1_pose(self):      self._send_r1_pose(urgent=False)

    def _publish_custom_msg_tick(self):
        with self.lock:
            x, y, z, yaw = self.x, self.y, self.z, self.yaw
            forest        = self.forest.copy()
            current_block = self.current_block
            r1_x, r1_y   = self.r1_x, self.r1_y
            r1_position  = self.r1_position
            r1_command   = self.r1_command
            lidar_ok     = self.lidar_ok
            camera_ok    = self.camera_ok
            end          = self.end
        self._publish_custom_msg(x, y, z, yaw, forest, current_block,
                                  r1_x, r1_y, r1_position, r1_command,
                                  lidar_ok, camera_ok, end)

    # ── CAN RX: H7 -> laptop ─────────────────────────────────────────────────

    def _on_h7_can(self, can_id, data):
        if can_id != CAN_ID_H7_FEEDBACK:
            return
        if len(data) < _R2OUT_SIZE:
            return
        enc_x, enc_y, r2_yaw, navi_state, completed = struct.unpack_from(
            _R2OUT_FMT, data
        )
        payload = json.dumps({
            'encoder_x':          enc_x,
            'encoder_y':          enc_y,
            'r2_yaw':             r2_yaw,
            'current_navi_state': navi_state,
            'completed':          bool(completed),
        })
        self.h7_state_pub.publish(String(data=payload))

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _position_to_block(self, x, y):
        BLOCK_1_X = 0.0
        BLOCK_1_Y = 0.0
        SPACING   = 1.2
        BLOCK_LAYOUT = {
            1:(0,0), 2:(1,0), 3:(2,0),
            4:(0,1), 5:(1,1), 6:(2,1),
            7:(0,2), 8:(1,2), 9:(2,2),
            10:(0,3), 11:(1,3), 12:(2,3),
        }
        best_block = 0
        best_dist  = float('inf')
        for block_num, (col, row) in BLOCK_LAYOUT.items():
            bx = BLOCK_1_X + col * SPACING
            by = BLOCK_1_Y + row * SPACING
            dist = math.sqrt((x - bx)**2 + (y - by)**2)
            if dist < best_dist:
                best_dist  = dist
                best_block = block_num
        if best_dist > 0.6:
            return 0
        return best_block

    def _publish_custom_msg(self, x, y, z, yaw, forest, current_block,
                             r1_x, r1_y, r1_position, r1_command,
                             lidar_ok, camera_ok, end):
        msg = CustomMsg()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.x = float(x); msg.y = float(y); msg.z = float(z); msg.yaw = float(yaw)
        msg.forest        = forest
        msg.current_block = current_block
        msg.r1_x = float(r1_x); msg.r1_y = float(r1_y)
        msg.r1_position   = r1_position
        msg.r1_command    = r1_command
        msg.lidar_ok      = lidar_ok
        msg.camera_ok     = camera_ok
        msg.end           = end
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.can_sender.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    aggregator_node = AggregatorNode()
    try:
        rclpy.spin(aggregator_node)
    except KeyboardInterrupt:
        pass
    finally:
        aggregator_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
