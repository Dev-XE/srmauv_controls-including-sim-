import sys
import struct

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel,
    QSlider, QPushButton, QHBoxLayout, QLineEdit
)
from PyQt5.QtCore import Qt

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


# ==============================
# CRC8 (same as Arduino)
# ==============================
def crc8_poly07(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def make_packet(param_id: int, value: float) -> bytes:
    value_bytes = struct.pack('<f', value)
    payload = bytes([param_id]) + value_bytes
    crc = crc8_poly07(payload)
    return payload + bytes([crc])


# ==============================
# ROS2 Node
# ==============================
class ParamPublisher(Node):
    def __init__(self):
        super().__init__('depth_param_gui')
        self.pub = self.create_publisher(UInt8MultiArray, 'nemo_auv/depth_params', 10)

    def send(self, param_id, value):
        pkt = make_packet(param_id, value)

        msg = UInt8MultiArray()
        msg.data = list(pkt)

        self.pub.publish(msg)

        self.get_logger().info(
            f"Sent ID=0x{param_id:02X}, value={value:.3f}, packet={list(pkt)}"
        )


# ==============================
# GUI
# ==============================
class PIDGui(QWidget):
    PARAMS = {
        "Kp": 0x01,
        "Kd": 0x02,
        "Ki": 0x03,
        "Offset": 0x04
    }

    def __init__(self, ros_node):
        super().__init__()

        self.node = ros_node

        self.setWindowTitle("AUV PID Tuner")
        self.setGeometry(200, 200, 400, 300)

        layout = QVBoxLayout()

        self.controls = {}

        for name, pid in self.PARAMS.items():
            layout.addLayout(self.create_param_control(name, pid))

        self.setLayout(layout)

    def create_param_control(self, name, param_id):
        layout = QVBoxLayout()

        label = QLabel(f"{name}: 0.0")
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(0)
        slider.setMaximum(1000)
        slider.setValue(0)

        textbox = QLineEdit("0.0")
        send_button = QPushButton("Send")

        def slider_changed():
            value = slider.value() / 100.0
            label.setText(f"{name}: {value:.2f}")
            textbox.setText(f"{value:.2f}")

        def send_value():
            try:
                value = float(textbox.text())
                self.node.send(param_id, value)
            except:
                print("Invalid value")

        slider.valueChanged.connect(slider_changed)
        send_button.clicked.connect(send_value)

        row = QHBoxLayout()
        row.addWidget(textbox)
        row.addWidget(send_button)

        layout.addWidget(label)
        layout.addWidget(slider)
        layout.addLayout(row)

        return layout


# ==============================
# Main
# ==============================
def main():
    rclpy.init()

    ros_node = ParamPublisher()

    app = QApplication(sys.argv)
    gui = PIDGui(ros_node)
    gui.show()

    # ROS + Qt loop
    timer = gui.startTimer(50)

    def timerEvent(event):
        rclpy.spin_once(ros_node, timeout_sec=0)

    gui.timerEvent = timerEvent

    sys.exit(app.exec_())

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()