"""
Operator-stack launcher.

r2_brain is the single planner/controller and all robot commands go over CAN via
aggregator_node. The old serial path (scripts/bridge.py → UART /dev/ttyUSB0) is
RETIRED and no longer launched.

Boot order:
  1. can0 up @ 1 Mbps   (aggregator_node ↔ H7; matches AggregatorNode defaults)
  2. rosbridge :9090     (the Flutter operator console connects here)
  3. aggregator_node     (ROS topics → CAN frames)
  4. r2_brain            (the BT / single planner)
  5. flutter run         (operator console)

The ROS nodes live in the Demo5_laptop workspace — source it before running
this (e.g. `source ~/RBC_workspace/DEMO5/Demo5_laptop/install/setup.bash`), or
set R2_WS to its install dir and this script will source it for you.
"""

import os
import threading
import time

R2_WS = os.environ.get("R2_WS")  # optional: Demo5_laptop install dir to source
_SOURCE = f"source {R2_WS}/setup.bash && " if R2_WS else ""


def bring_up_can():
    """Best-effort can0 bring-up @ 1 Mbps (AggregatorNode uses can0/1000000).

    Needs privileges; if it fails (no perms / no CAN adapter) we warn and carry
    on — AggregatorNode degrades gracefully when CAN init fails.
    """
    print("[Launcher] Bringing up can0 @ 1 Mbps...")
    # Idempotent: set type+bitrate only when the link is down, then bring it up.
    os.system(
        "sudo ip link set can0 type can bitrate 1000000 2>/dev/null; "
        "sudo ip link set can0 up 2>/dev/null && echo '[Launcher] can0 up' "
        "|| echo '[Launcher] WARN: could not bring up can0 (perms/no adapter) — "
        "aggregator_node will run with CAN disabled'"
    )


def run_rosbridge():
    print("[Launcher] Starting ROSbridge (Port 9090)...")
    os.system("ros2 launch rosbridge_server rosbridge_websocket_launch.xml")


def run_aggregator():
    print("[Launcher] Starting aggregator_node (ROS → CAN)...")
    os.system(f"{_SOURCE}ros2 run aggregator_node aggregator_node")


def run_r2_brain():
    print("[Launcher] Starting r2_brain (BT / single planner)...")
    os.system(f"{_SOURCE}ros2 run r2_brain r2_brain")


def run_flutter():
    print("[Launcher] Starting Flutter operator console...")
    os.system("flutter run")


if __name__ == "__main__":
    # 1. CAN link
    bring_up_can()

    # 2. rosbridge (operator console transport)
    threading.Thread(target=run_rosbridge, daemon=True).start()
    time.sleep(2)

    # 3. aggregator_node (ROS topics → CAN)
    threading.Thread(target=run_aggregator, daemon=True).start()
    time.sleep(1)

    # 4. r2_brain (the brain)
    threading.Thread(target=run_r2_brain, daemon=True).start()
    time.sleep(1)

    # 5. operator console (foreground)
    run_flutter()
