import rclpy
import pygame
from rclpy.node import Node
from nemo_interfaces.msg import RovCommands
import serial
import time
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray,Int8MultiArray

START_BIT = 0xAA
STOP_BIT = 0x55



class ThrusterSerialNode(Node):
    def __init__(self):
        super().__init__('thruster_serial_node')

        self.declare_parameter('SerialPort', '/dev/ttyUSB0')
        self.serial_port = self.get_parameter('SerialPort').get_parameter_value().string_value

        self.arming_state = False
        
        try:
            self.ser = serial.Serial(self.serial_port, 115200, timeout=0.1)
            time.sleep(2)
            self.get_logger().info("✅ Serial port connected.")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Serial open failed: {e}")
            self.ser = None

        self.subscription_cmd = self.create_subscription(
            Int8MultiArray, '/nemo_auv/thruster_vals', self.cmd_callback, 10
        )

        """self.subscription_joy = self.create_subscription(
            Bool,'/arm_status',self.joy_callback, 10
        )"""

        # Timer: checks joystick state every 50ms (non-blocking)

    def joy_callback(self,msg):

        msg = Bool()
        state = msg.data
        if state:
            self.arming_state = not self.arming_state 
    
        

    def cmd_callback(self, msg):
        """Send thruster data only."""
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial not available.")
            return
        

        arr = msg.data

        output_vals = []
        for i in range(0,8):
            mapped_val = arr[i] 
            output_vals.append(mapped_val & 0xFF)

        if self.arming_state:
            output_vals = [0,0,0,0,0,0,0,0]
        

        packet = bytearray([0xAA,output_vals[0],output_vals[1],output_vals[2],output_vals[3],
                            output_vals[4],output_vals[5],output_vals[6],output_vals[7],0x55])
        
        try:
            self.ser.write(packet)
            print(f"written packet <{packet}>")
        except Exception as e:
            self.get_logger().error(f"❌ Serial write failed: {e}")

    def map(self,variable,abs_in,abs_out):

        return_val = ((variable)/abs_in)*abs_out

        return return_val
        


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