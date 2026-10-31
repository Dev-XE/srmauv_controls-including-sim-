#!/usr/bin/env python3
import math
import struct
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

START_BYTE = 0xAA
END_BYTE = 0x55
PACKET_SIZE = 68

# Layout (little-endian):
# 0   : start (B)
# 1-4 : time_stamp (uint32)
# 5.. : 14 floats? (NO) -> Your packet has 15 float fields after timestamp:
# gyro(3), accel(3), mag(3), quat(4), depth(1), depth_rate(1) = 15 floats
# Total payload after time: 15 * 4 = 60 bytes
# End: end byte + '\r' + '\n'
#
# So overall:
# start(1) + u32(4) + 15f(60) + end(1) + crlf(2) = 68

PKT_STRUCT = struct.Struct("<BI15fBcc")


def u32_delta(new: int, old: int) -> int:
    return (new - old) & 0xFFFFFFFF


class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_py')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.0)
            try:
                self.ser.dtr = False
                self.ser.rts = False
            except Exception:
                pass
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().fatal(f"Failed to open serial port: {e}")
            rclpy.shutdown()
            return

        self.imu_pub = self.create_publisher(Imu, "/nemo_auv/imu/data_raw", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Float32, "/nemo_auv/depth", qos_profile_sensor_data)
        self.depth_rate_pub = self.create_publisher(Float32, "/nemo_auv/depth_rate", qos_profile_sensor_data)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()

        self.rx = bytearray()

        # millis->ROS time mapping
        self.have_anchor = False
        self.anchor_ros_ns = 0
        self.anchor_ms = 0
        self.last_stamp_ns = 0

        self.timer = self.create_timer(0.002, self.read_and_publish)

        self.get_logger().info(f"IMU Serial Node started on {self.port} @ {self.baud}")

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp.sec = 0
        t.header.stamp.nanosec = 0
        t.header.frame_id = "base_link"
        t.child_frame_id = "imu_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def ms_to_stamp(self, t_ms: int):
        now = self.get_clock().now()
        now_ns = int(now.nanoseconds)

        if not self.have_anchor:
            self.have_anchor = True
            self.anchor_ros_ns = now_ns
            self.anchor_ms = t_ms
            self.last_stamp_ns = now_ns
            return now.to_msg()

        dt_ms = u32_delta(t_ms, self.anchor_ms)
        stamp_ns = self.anchor_ros_ns + int(dt_ms) * 1_000_000

        # enforce monotonic
        if stamp_ns <= self.last_stamp_ns:
            stamp_ns = self.last_stamp_ns + 1_000_000
        self.last_stamp_ns = stamp_ns

        sec = stamp_ns // 1_000_000_000
        nsec = stamp_ns % 1_000_000_000
        msg = now.to_msg()
        msg.sec = int(sec)
        msg.nanosec = int(nsec)
        return msg

    def _extract_one(self):
        if len(self.rx) < PACKET_SIZE:
            return None

        start_idx = self.rx.find(bytes([START_BYTE]))
        if start_idx < 0:
            self.rx.clear()
            return None

        if start_idx > 0:
            del self.rx[:start_idx]
            if len(self.rx) < PACKET_SIZE:
                return None

        # Quick end/crlf check at fixed offsets:
        # end is at index 65, \r at 66, \n at 67
        if self.rx[65] != END_BYTE or self.rx[66] != 0x0D or self.rx[67] != 0x0A:
            # false start, slide by 1
            del self.rx[0:1]
            return "progress"

        pkt = bytes(self.rx[:PACKET_SIZE])
        del self.rx[:PACKET_SIZE]

        try:
            start, t_ms, *rest = PKT_STRUCT.unpack(pkt)
        except struct.error:
            return "progress"

        end = rest[-2]
        if start != START_BYTE or end != END_BYTE:
            return "progress"

        floats = rest[0:15]
        return t_ms, floats

    def read_and_publish(self):
        data = self.ser.read(2048)
        if data:
            self.rx.extend(data)

        for _ in range(50):
            out = self._extract_one()
            if out is None:
                break
            if out == "progress":
                continue

            t_ms, f = out

            gx, gy, gz = f[0], f[1], f[2]
            ax, ay, az = f[3], f[4], f[5]
            mx, my, mz = f[6], f[7], f[8]
            qw, qx, qy, qz = f[9], f[10], f[11], f[12]
            depth, depth_rate = f[13], f[14]

            stamp = self.ms_to_stamp(int(t_ms))

            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = "imu_link"

            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz

            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az

            # Normalize quaternion
            n2 = qw*qw + qx*qx + qy*qy + qz*qz
            if not math.isfinite(n2) or n2 < 1e-12:
                imu.orientation.w = 1.0
                imu.orientation.x = 0.0
                imu.orientation.y = 0.0
                imu.orientation.z = 0.0
            else:
                inv = 1.0 / math.sqrt(n2)
                imu.orientation.w = qw * inv
                imu.orientation.x = qx * inv
                imu.orientation.y = qy * inv
                imu.orientation.z = qz * inv

            # If unknown, advertise unknown covariances
            imu.orientation_covariance[0] = -1.0
            imu.angular_velocity_covariance[0] = -1.0
            imu.linear_acceleration_covariance[0] = -1.0

            self.imu_pub.publish(imu)

            d = Float32()
            d.data = float(depth)
            self.depth_pub.publish(d)

            dr = Float32()
            dr.data = float(depth_rate)
            self.depth_rate_pub.publish(dr)


def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
