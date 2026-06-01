#!/usr/bin/env python3
"""
Demo script: send fake classic-CAN frames matching the new aggregator
protocol so the STM32 can be exercised without the full ROS stack.

Frames sent (all classic CAN, 8 bytes each):
  0x101 CRITICAL_CMD — r2_h_cmd + target_x/y/yaw_cd + seq    (~20 Hz)
  0x102 ZONES        — r2_zone + r1_zone + navi + seq         (~10 Hz)
  0x103 R2_POSE      — x/y/z_mm + yaw_cd                       (~20 Hz)
  0x104 R1_POSE      — r1_x/y_mm                               (~5 Hz)

It also fires an E_STOP burst halfway through (and one new-target event)
so you can verify the priority path: those frames should reach the H7
ahead of any queued lower-priority frames.

Usage:
    python3 demo_can_sender.py [can_channel]   e.g. can0 (default)

Prerequisite — bring up can0 in classic-CAN mode:
    sudo ip link set can0 down
    sudo ip link set can0 up type can bitrate 1000000
"""

import can
import struct
import time
import math
import sys

# ── CAN IDs (must match AggregatorNode.py) ───────────────────────────────────

CAN_ID_CRITICAL_CMD = 0x101
CAN_ID_ZONES        = 0x102
CAN_ID_R2_POSE      = 0x103
CAN_ID_R1_POSE      = 0x104
CAN_ID_H7_FEEDBACK  = 0x110

# Zone_t enum
ZONE_MARTIAL_CLUB = 0
ZONE_FOREST       = 1
ZONE_ARENA        = 2

# R2_Cmd_t enum
CMD_GO       = 0
CMD_PICK     = 1
CMD_CONTINUE = 2
CMD_E_STOP   = 3

# Frame periods (seconds) — match the aggregator
PERIOD_CRITICAL_CMD = 0.05    # 20 Hz
PERIOD_ZONES        = 0.10    # 10 Hz
PERIOD_R2_POSE      = 0.05    # 20 Hz
PERIOD_R1_POSE      = 0.20    # 5 Hz

# R2Outgoing_t (inbound 0x110)
_R2OUT_FMT  = '<IIHBB'
_R2OUT_SIZE = struct.calcsize(_R2OUT_FMT)   # 12 bytes


# ── Frame packers (mirror AggregatorNode.pack_*) ─────────────────────────────

def clamp16(v):
    return max(-32768, min(32767, int(v)))


def pack_critical_cmd(r2_h_cmd, tx_mm, ty_mm, tyaw_cd, seq):
    return struct.pack('<BhhhB',
                       r2_h_cmd & 0xFF,
                       tx_mm, ty_mm, tyaw_cd,
                       seq & 0xFF)


def pack_zones(r2_zone, r1_zone, navi, seq):
    return struct.pack('<BBBB4x',
                       r2_zone & 0xFF, r1_zone & 0xFF,
                       navi & 0xFF, seq & 0xFF)


def pack_r2_pose(x_mm, y_mm, z_mm, yaw_cd):
    return struct.pack('<hhhh', x_mm, y_mm, z_mm, yaw_cd)


def pack_r1_pose(r1_x_mm, r1_y_mm):
    return struct.pack('<hh4x', r1_x_mm, r1_y_mm)


# ── Dummy data generator ─────────────────────────────────────────────────────

def make_state(t):
    """Generate a slowly-varying scenario so values are easy to verify on H7."""
    # R2 pose: slow circle
    x   = 1.5 * math.cos(t * 0.3)
    y   = 1.5 * math.sin(t * 0.3)
    z   = 0.0
    yaw = math.fmod(t * 0.5, 2 * math.pi) - math.pi

    # R1 pose: stationary
    r1_x, r1_y = 0.5, -0.5

    # Zone cycles every 10 s
    r2_zone  = int(t / 10) % 3
    r1_zone  = ZONE_MARTIAL_CLUB
    navi     = (int(t / 5) % 4)

    # Target: ahead of current pose
    tx, ty, tyaw = x + 0.3, y + 0.3, yaw

    # Default command
    cmd = CMD_GO

    # ---- Inject scripted events to exercise URGENT priority -------------
    # 5 s in: change target dramatically (urgent on AggregatorNode side)
    if 5.0 <= t < 5.5:
        tx, ty, tyaw = 2.5, -1.0, math.pi / 2
    # 10 s in: E_STOP burst for 0.5 s
    if 10.0 <= t < 10.5:
        cmd = CMD_E_STOP
    # 15 s in: PICK command
    if 15.0 <= t < 15.5:
        cmd = CMD_PICK

    return {
        'cmd': cmd, 'navi': navi,
        'r2_zone': r2_zone, 'r1_zone': r1_zone,
        'x': x, 'y': y, 'z': z, 'yaw': yaw,
        'r1_x': r1_x, 'r1_y': r1_y,
        'tx': tx, 'ty': ty, 'tyaw': tyaw,
    }


# ── Optional: pretty-print any H7 reply frames ───────────────────────────────

def print_h7_reply(msg):
    if msg.arbitration_id != CAN_ID_H7_FEEDBACK:
        print(f"  [RX  ] 0x{msg.arbitration_id:03X}  {bytes(msg.data).hex()}")
        return
    if len(msg.data) < _R2OUT_SIZE:
        return
    enc_x, enc_y, r2_yaw, navi_state, completed = struct.unpack_from(_R2OUT_FMT, msg.data)
    print(f"  [RX H7] 0x{msg.arbitration_id:03X}  "
          f"enc=({enc_x},{enc_y}) yaw={r2_yaw} navi={navi_state} done={bool(completed)}")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    channel = sys.argv[1] if len(sys.argv) > 1 else 'can0'
    bitrate = 1000000

    print(f"Opening CAN interface: {channel}  bitrate={bitrate}  (classic CAN)")
    print("  Bring up first with:")
    print("    sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000")

    try:
        bus = can.interface.Bus(channel=channel, interface='socketcan',
                                bitrate=bitrate)
        print("CAN interface ready.\n")
    except Exception as e:
        print(f"Failed to open CAN interface: {e}")
        sys.exit(1)

    print("Streaming dummy frames. Scripted events:")
    print("   t=5s   target jumps to (2.50,-1.00, 90 deg)")
    print("   t=10s  E_STOP for 0.5 s")
    print("   t=15s  PICK for 0.5 s")
    print("Press Ctrl+C to stop.\n")

    seq_cmd = 0
    seq_zones = 0

    # Per-frame "due time"
    t0 = time.monotonic()
    next_cmd   = t0
    next_zones = t0
    next_r2    = t0
    next_r1    = t0

    # Track previous values so we can fire urgent frames on change
    prev_cmd_payload   = None
    prev_zones_payload = None

    total_ok = total_fail = 0

    def tx(can_id, data, tag=' '):
        nonlocal total_ok, total_fail
        try:
            msg = can.Message(arbitration_id=can_id, data=data,
                              is_extended_id=False)
            t_send = time.monotonic()
            bus.send(msg, timeout=0.05)
            dt_ms = (time.monotonic() - t_send) * 1000
            total_ok += 1
            print(f"[TX{tag}] 0x{can_id:03X} {len(data)}B  ACK {dt_ms:.2f} ms  "
                  f"data={data.hex()}")
        except Exception as e:
            total_fail += 1
            print(f"[TX ] 0x{can_id:03X} FAIL: {e}")

    try:
        while True:
            now = time.monotonic()
            t = now - t0
            st = make_state(t)

            # ── 0x101 CRITICAL_CMD ─────────────────────────────────────
            seq_cmd = (seq_cmd + 1) & 0xFF
            cmd_payload = pack_critical_cmd(
                st['cmd'],
                clamp16(st['tx'] * 1000),
                clamp16(st['ty'] * 1000),
                clamp16(math.degrees(st['tyaw']) * 100),
                seq_cmd,
            )
            # ignore seq byte when comparing for "change"
            changed = (prev_cmd_payload is None or
                       cmd_payload[:-1] != prev_cmd_payload[:-1])
            if changed:
                tx(CAN_ID_CRITICAL_CMD, cmd_payload, tag='!')   # urgent
                next_cmd = now + PERIOD_CRITICAL_CMD
                prev_cmd_payload = cmd_payload
            elif now >= next_cmd:
                tx(CAN_ID_CRITICAL_CMD, cmd_payload, tag=' ')   # periodic
                next_cmd = now + PERIOD_CRITICAL_CMD
                prev_cmd_payload = cmd_payload

            # ── 0x102 ZONES ────────────────────────────────────────────
            seq_zones = (seq_zones + 1) & 0xFF
            zones_payload = pack_zones(st['r2_zone'], st['r1_zone'],
                                       st['navi'], seq_zones)
            changed = (prev_zones_payload is None or
                       zones_payload[:-1] != prev_zones_payload[:-1])
            if changed:
                tx(CAN_ID_ZONES, zones_payload, tag='!')
                next_zones = now + PERIOD_ZONES
                prev_zones_payload = zones_payload
            elif now >= next_zones:
                tx(CAN_ID_ZONES, zones_payload, tag=' ')
                next_zones = now + PERIOD_ZONES
                prev_zones_payload = zones_payload

            # ── 0x103 R2_POSE ──────────────────────────────────────────
            if now >= next_r2:
                tx(CAN_ID_R2_POSE,
                   pack_r2_pose(
                       clamp16(st['x'] * 1000),
                       clamp16(st['y'] * 1000),
                       clamp16(st['z'] * 1000),
                       clamp16(math.degrees(st['yaw']) * 100),
                   ), tag=' ')
                next_r2 = now + PERIOD_R2_POSE

            # ── 0x104 R1_POSE ──────────────────────────────────────────
            if now >= next_r1:
                tx(CAN_ID_R1_POSE,
                   pack_r1_pose(
                       clamp16(st['r1_x'] * 1000),
                       clamp16(st['r1_y'] * 1000),
                   ), tag=' ')
                next_r1 = now + PERIOD_R1_POSE

            # Print scenario summary roughly every second
            if int(t * 4) != int((t - 0.025) * 4):
                cmd_name = {0:'GO',1:'PICK',2:'CONT',3:'E_STOP'}.get(st['cmd'], '?')
                print(f"  t={t:5.1f}s  cmd={cmd_name}  zone={st['r2_zone']}  "
                      f"pos=({st['x']:.2f},{st['y']:.2f})  "
                      f"target=({st['tx']:.2f},{st['ty']:.2f}) "
                      f"yaw={math.degrees(st['tyaw']):.0f}deg  "
                      f"OK={total_ok} FAIL={total_fail}")

            # Drain any pending H7 reply frames
            while True:
                reply = bus.recv(timeout=0.0)
                if reply is None:
                    break
                print_h7_reply(reply)

            # Sleep until the next due frame
            sleep_s = min(next_cmd, next_zones, next_r2, next_r1) - time.monotonic()
            if sleep_s > 0:
                time.sleep(min(sleep_s, 0.02))

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        bus.shutdown()
        print("CAN interface closed.")


if __name__ == '__main__':
    main()
