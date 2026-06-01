# Aggregator Node Demo

This package aggregates data from various ROS nodes and sends it to the mainboard via CAN bus.

## Demo CAN Sender

The `demo_can_sender.py` script sends fake CAN data using the same message format and IDs as the aggregator node. This allows testing the CAN receiving functionality without setting up the full ROS system.

### Usage

**For hardware CAN (with CAN-to-USB adapter):**
```bash
python3 demo_can_sender.py  # Uses can0 by default
```

**For virtual CAN testing:**
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
python3 demo_can_sender.py vcan0
```

**To monitor CAN traffic:**
```bash
candump can0  # See all CAN messages
candump can0 -x  # With extended info
```

3. The script will send fake data every second with random values for:
   - Robot position (x, y, z, yaw)
   - Forest grid (12 blocks with KFS markers)
   - Current block
   - R1 position
   - System status flags

### CAN Message Format

The script sends messages with the following IDs:

- `0x100`: Position data (x, y, z, yaw)
- `0x105`: Forest blocks 1-8
- `0x106`: Forest blocks 9-12, current block, status flags
- `0x107`: R1 position

Data is packed using the same struct formats as the main aggregator node.

### Requirements

- python-can library (`pip install python-can`)
- CAN interface configured and up (use vcan0 for virtual testing)