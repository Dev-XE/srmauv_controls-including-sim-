import rclpy
from rclpy.node import Node
import serial
import struct
import math

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from rclpy.qos import qos_profile_sensor_data

START_BYTE  = 0xAA
STOP_BYTE   = 0x55
PACKET_SIZE = 52 

class IMUPacketNode(Node):
    def __init__(self):
        super().__init__('imu_packet_node')

        # ---------- Serial ----------
        self.ser = serial.Serial(port='/dev/ttyUSB0',baudrate=115200,timeout=0.0)
        self.buffer = bytearray()
        # Publisher
        self.imu_pub = self.create_publisher(Imu,'/imu/data_raw',qos_profile_sensor_data)
        # magnetic field
        self.mag_pub = self.create_publisher(MagneticField,'/imu/mag',qos_profile_sensor_data)
        # ---------- Rate limiting ----------
        self.publish_hz = 50.0
        self.publish_period_ns = int(1e9 / self.publish_hz)
        self.last_pub_time = self.get_clock().now()
        # ---------- Timer ----------
        self.timer = self.create_timer(0.01, self.read_serial)
        self.get_logger().info("Custom ESP32 IMU serial node started")

    def read_serial(self):
        data = self.ser.read(128)
        if not data:
            return

        self.buffer.extend(data)

        while len(self.buffer) >= PACKET_SIZE:

            # Sync to START byte
            if self.buffer[0] != START_BYTE:
                self.buffer.pop(0)
                continue

            packet = self.buffer[:PACKET_SIZE]

            # Validate STOP byte
            if packet[37] != STOP_BYTE:
                self.buffer.pop(0)
                continue

            # ---------- Decode ----------
            pitch      = struct.unpack('<f', packet[1:5])[0]
            pitch_rate = struct.unpack('<f', packet[5:9])[0]

            roll       = struct.unpack('<f', packet[9:13])[0]
            roll_rate  = struct.unpack('<f', packet[13:17])[0]

            yaw = struct.unpack('<f', packet[17:21])[0]
            yaw_rate = struct.unpack('<f', packet[21:25])[0]

            accel_x = struct.unpack('<f', packet[25:29])[0]
            accel_y = struct.unpack('<f', packet[29:33])[0]
            accel_z = struct.unpack('<f', packet[33:37])[0]

            

            # OPTIONAL: uncomment if ESP sends degrees
            # pitch = math.radians(pitch)
            # roll  = math.radians(roll)

            # ---------- Throttle publishing ----------
            now = self.get_clock().now()
            if (now - self.last_pub_time).nanoseconds < self.publish_period_ns:
                self.buffer = self.buffer[PACKET_SIZE:]
                continue
            self.last_pub_time = now

            # ---------- Orientation ----------
            qx, qy, qz, qw = self.euler_to_quaternion(math.radians(roll), math.radians(pitch), math.radians(yaw))

            msg = Imu()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = "imu_link"

            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

            msg.angular_velocity.x = roll_rate
            msg.angular_velocity.y = pitch_rate
            msg.angular_velocity.z = yaw_rate

            # Covariances (reasonable defaults)
            msg.orientation_covariance = [
                0.01, 0.0,  0.0,
                0.0,  0.01, 0.0,
                0.0,  0.0,  999.0
            ]

            msg.angular_velocity_covariance = [
                0.02, 0.0,  0.0,
                0.0,  0.02, 0.0,
                0.0,  0.0,  999.0
            ]

            # No linear acceleration
            var = 0.01
            msg.linear_acceleration_covariance = [
                var, 0.0, 0.0,
                0.0, var, 0.0,
                0.0, 0.0, var
            ]


            msg.linear_acceleration.x = accel_x
            msg.linear_acceleration.y = accel_y
            msg.linear_acceleration.z = accel_z

            
            self.imu_pub.publish(msg)

            mag_msg = MagneticField()

            # Remove processed packet
            self.buffer = self.buffer[PACKET_SIZE:]

    def euler_to_quaternion(self, roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw


def main():
    rclpy.init()
    node = IMUPacketNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
