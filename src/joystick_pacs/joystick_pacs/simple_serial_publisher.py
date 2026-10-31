import rclpy
from rclpy.node import Node
from nemo_interfaces.msg import RovCommands
import serial
import time

START_BIT = 0xAA
STOP_BIT  = 0x55

def to_int8_byte(x: float) -> int:
    """
    Converts normalized float (-1..1) to signed int8 (-100..100),
    then packs into a single byte (0..255) using two's complement.
    """
    v = int(round(x * 100.0))
    if v > 100:
        v = 100
    if v < -100:
        v = -100
    return v & 0xFF

class ThrusterSerialNode(Node):
    def __init__(self):
        super().__init__('thruster_serial_node')

        self.declare_parameter('SerialPort', '/dev/ttyUSB0')
        self.declare_parameter('Baud', 115200)

        self.serial_port = self.get_parameter('SerialPort').get_parameter_value().string_value
        self.baud = self.get_parameter('Baud').get_parameter_value().integer_value

        self.ser = None
        try:
            self.ser = serial.Serial(self.serial_port, self.baud, timeout=0.05, write_timeout=0.05)
            time.sleep(0.3)
            self.get_logger().info(f"✅ Serial connected: {self.serial_port} @ {self.baud}")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Serial open failed: {e}")

        self.subscription_cmd = self.create_subscription(
            RovCommands, '/nemo_auv/input_cmd', self.cmd_callback, 10
        )

        # Optional: track last send time
        self.last_send_ns = 0

    def cmd_callback(self, msg: RovCommands):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial not available.")
            return

        surge_b = to_int8_byte(msg.surge)
        sway_b  = to_int8_byte(msg.sway)
        yaw_b   = to_int8_byte(msg.yaw)
        heave_b = to_int8_byte(msg.heave)

        if msg.surge > msg.yaw:
            t1_cmd = 1000 + int(abs(msg.surge * 1000))
            t2_cmd = 1000 + int(abs(msg.surge * 1000))
        else:
            if msg.surge > 0:
                t1_cmd = 1000
                t2_cmd = 1000 + int(abs(msg.yaw*1000))
            else:
                t1_cmd = 1000 + int(abs(msg.yaw * 1000))
                t2_cmd = 1000
        
        t3_cmd = 1000 + int(abs(msg.heave * 1000))
        t4_cmd = 1000 + int(abs(msg.heave * 1000))
        thruster_list = [t1_cmd,t2_cmd,t2_cmd,t3_cmd,t4_cmd]

        for i in range(len(thruster_list)):
            thruster_list[i] = int(thruster_list[i]-1000/10) & 0xFF
            

        packet = bytes([START_BIT, thruster_list[0],thruster_list[1],thruster_list[2],thruster_list[3], STOP_BIT])

        try:
            self.ser.write(packet)
            # self.ser.flush()  # usually not needed; uncomment if you want stricter timing
            # self.get_logger().debug(f"Sent: {list(packet)}")
        except serial.SerialTimeoutException:
            self.get_logger().warn("⚠️ Serial write timeout")
        except Exception as e:
            self.get_logger().error(f"❌ Serial write failed: {e}")

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
