#!/usr/bin/env python3
import sys
import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
from PyQt5.QtCore import QTimer


class ArmGuiNode(Node):
    def __init__(self):
        super().__init__('arm_gui_node')
        self.publisher_ = self.create_publisher(Bool, '/nemo_auv/arm', 10)
        self.get_logger().info('Arm/Disarm GUI node started')

    def publish_arm(self, state: bool):
        msg = Bool()
        msg.data = state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Arm state published: {state}')


class ArmWindow(QWidget):
    def __init__(self, ros_node: ArmGuiNode):
        super().__init__()
        self.node = ros_node
        self.armed = False

        self.setWindowTitle("AUV Arm / Disarm")
        self.setMinimumSize(300, 200)

        self.button = QPushButton("DISARMED")
        self.button.setStyleSheet(
            "font-size: 28px; padding: 30px; background-color: red; color: white;"
        )
        self.button.clicked.connect(self.toggle)

        layout = QVBoxLayout()
        layout.addWidget(self.button)
        self.setLayout(layout)

    def toggle(self):
        self.armed = not self.armed
        self.node.publish_arm(self.armed)

        if self.armed:
            self.button.setText("ARMED")
            self.button.setStyleSheet(
                "font-size: 28px; padding: 30px; background-color: green; color: white;"
            )
        else:
            self.button.setText("DISARMED")
            self.button.setStyleSheet(
                "font-size: 28px; padding: 30px; background-color: red; color: white;"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArmGuiNode()

    app = QApplication(sys.argv)
    window = ArmWindow(node)
    window.show()

    # ROS spin inside Qt loop
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    ros_timer.start(10)

    # Handle Ctrl+C -> close Qt cleanly
    def handle_sigint(signum, frame):
        app.quit()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        exit_code = app.exec_()
    finally:
        # Clean shutdown sequence
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
