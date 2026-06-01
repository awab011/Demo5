# RETIRED — do not launch.
#
# This was the serial path: it subscribed to /robot_paths and wrote binary CAN
# frames over UART (/dev/ttyUSB0) to the STM32. As of the r2_brain integration,
# all robot commands go over CAN via aggregator_node (ROS → can0 → FDCAN3), and
# r2_brain is the single planner. start.py no longer launches this file. Kept
# for reference only; do not re-add it to the launch sequence.
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')
        #chg to actual usb port
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.01)
        
        #subscribe to path from phone
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/robot_paths',
            self.listener_callback,
            10)
        
        self.timer = self.create_timer(0.01, self.check_serial_rx)
        self.get_logger().info('Bridge started. Waiting for phone path...')

    def listener_callback(self, msg):
        #take binary from phone and send to STM32
        raw_bytes = bytes(msg.data)
        self.ser.write(raw_bytes)
        self.get_logger().info(f'Sent {len(raw_bytes)} binary bytes to STM32')

    def check_serial_rx(self):
        #listen data frm mb
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    print(f"\033[92m[ROBOT DEBUG]: {line}\033[0m") #green
            except Exception:
                pass 

def main(args=None):
    rclpy.init(args=args)
    bridge = RobotBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()