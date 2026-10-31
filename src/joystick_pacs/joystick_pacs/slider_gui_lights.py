#!/usr/bin/env python3
import sys
import signal
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int16MultiArray
from geometry_msgs.msg import Twist
from nemo_interfaces.msg import RovCommands

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QPushButton,
    QVBoxLayout,
    QSlider,
    QLabel,
)
from PyQt5.QtCore import Qt, QTimer


class ArmGuiNode(Node):
    def __init__(self):
        super().__init__('arm_gui_node')

        # Thruster arm publisher
        self.arm_pub = self.create_publisher(Bool, '/nemo_auv/arm', 10)

        # Manipulator arm publisher
        self.manip_arm_pub = self.create_publisher(Bool, '/nemo_auv/manipulator_arm', 10)

        # Cmd_vel publisher — forwards joystick commands to the Teensy
        self.cmd_vel_pub = self.create_publisher(Twist, '/nemo_auv/cmd_vel', 10)

        # Light publisher — sends brightness array to the Teensy
        self.light_pub = self.create_publisher(Int16MultiArray, '/nemo_auv/lights', 10)

        # Subscribe to custom joystick commands
        self.input_cmd_sub = self.create_subscription(
            RovCommands,
            '/nemo_auv/input_cmd',
            self.input_cmd_callback,
            10
        )

        # Internal states
        self.thruster_armed_state = False
        self.manip_armed_state = False
        self.light1_val = 0
        self.light2_val = 0

        # Timers
        self.heartbeat_timer = self.create_timer(0.5, self.publish_heartbeat) # 2 Hz for arming
        self.light_timer = self.create_timer(0.1, self.publish_lights)        # 10 Hz for lights

        self.last_log_time = 0.0

        self.get_logger().info('System Control GUI node started. Waiting for /nemo_auv/input_cmd...')

    def set_thruster_arm(self, state: bool):
        self.thruster_armed_state = state
        self.get_logger().info(f'>> GUI Button Clicked - Thrusters state set to: {state}')
        self.publish_heartbeat()

    def set_manip_arm(self, state: bool):
        self.manip_armed_state = state
        self.get_logger().info(f'>> GUI Button Clicked - Manipulator state set to: {state}')
        self.publish_heartbeat()

    def publish_heartbeat(self):
        t_msg = Bool()
        t_msg.data = self.thruster_armed_state
        self.arm_pub.publish(t_msg)

        m_msg = Bool()
        m_msg.data = self.manip_armed_state
        self.manip_arm_pub.publish(m_msg)

    def publish_lights(self):
        l_msg = Int16MultiArray()
        l_msg.data = [self.light1_val, self.light2_val]
        self.light_pub.publish(l_msg)

    def input_cmd_callback(self, msg: RovCommands):
        # Drop commands when thrusters are disarmed — don't feed the Teensy
        if not self.thruster_armed_state:
            return

        # Map RovCommands fields → Twist fields (Standard ROS coordinate frame)
        out_msg = Twist()
        out_msg.linear.x = float(msg.surge)   
        out_msg.linear.y = float(msg.sway)    
        out_msg.linear.z = float(msg.heave)   
        
        out_msg.angular.x = float(msg.roll)   
        out_msg.angular.y = float(msg.pitch)  
        out_msg.angular.z = float(msg.yaw)    

        self.cmd_vel_pub.publish(out_msg)

        current_time = time.time()
        if current_time - self.last_log_time > 2.0:
            self.get_logger().info(
                f"Forwarding Thruster Cmds -> S: {out_msg.linear.x:.2f}, "
                f"SW: {out_msg.linear.y:.2f}, H: {out_msg.linear.z:.2f}, "
                f"R: {out_msg.angular.x:.2f}, P: {out_msg.angular.y:.2f}, Y: {out_msg.angular.z:.2f}"
            )
            self.last_log_time = current_time


class ArmWindow(QWidget):
    def __init__(self, ros_node: ArmGuiNode):
        super().__init__()
        self.node = ros_node
        self.thruster_armed = False
        self.manip_armed = False

        self.setWindowTitle("AUV System Control")
        self.setMinimumSize(450, 450)

        layout = QVBoxLayout()

        # ==========================================
        # ARMING BUTTONS
        # ==========================================
        self.thruster_button = QPushButton("THRUSTERS: DISARMED")
        self.thruster_button.setStyleSheet(
            "font-size: 24px; padding: 30px; background-color: red; color: white; margin-bottom: 5px;"
        )
        self.thruster_button.clicked.connect(self.toggle_thruster_arm)

        self.manip_button = QPushButton("MANIPULATOR: DISARMED")
        self.manip_button.setStyleSheet(
            "font-size: 24px; padding: 30px; background-color: red; color: white; margin-bottom: 15px;"
        )
        self.manip_button.clicked.connect(self.toggle_manip_arm)

        layout.addWidget(self.thruster_button)
        layout.addWidget(self.manip_button)

        # ==========================================
        # LIGHT CONTROLS
        # ==========================================
        self.light1_label = QLabel("Front Light Intensity: 0")
        self.light1_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        self.light1_slider = QSlider(Qt.Horizontal)
        self.light1_slider.setRange(0, 255)
        self.light1_slider.valueChanged.connect(self.update_light1)

        self.light2_label = QLabel("Bottom Light Intensity: 0")
        self.light2_label.setStyleSheet("font-size: 16px; font-weight: bold; margin-top: 10px;")
        self.light2_slider = QSlider(Qt.Horizontal)
        self.light2_slider.setRange(0, 255)
        self.light2_slider.valueChanged.connect(self.update_light2)

        layout.addWidget(self.light1_label)
        layout.addWidget(self.light1_slider)
        layout.addWidget(self.light2_label)
        layout.addWidget(self.light2_slider)

        self.setLayout(layout)

    def toggle_thruster_arm(self):
        self.thruster_armed = not self.thruster_armed
        self.node.set_thruster_arm(self.thruster_armed)

        if self.thruster_armed:
            self.thruster_button.setText("THRUSTERS: ARMED")
            self.thruster_button.setStyleSheet(
                "font-size: 24px; padding: 30px; background-color: green; color: white; margin-bottom: 5px;"
            )
        else:
            self.thruster_button.setText("THRUSTERS: DISARMED")
            self.thruster_button.setStyleSheet(
                "font-size: 24px; padding: 30px; background-color: red; color: white; margin-bottom: 5px;"
            )

    def toggle_manip_arm(self):
        self.manip_armed = not self.manip_armed
        self.node.set_manip_arm(self.manip_armed)

        if self.manip_armed:
            self.manip_button.setText("MANIPULATOR: ARMED")
            self.manip_button.setStyleSheet(
                "font-size: 24px; padding: 30px; background-color: green; color: white; margin-bottom: 15px;"
            )
        else:
            self.manip_button.setText("MANIPULATOR: DISARMED")
            self.manip_button.setStyleSheet(
                "font-size: 24px; padding: 30px; background-color: red; color: white; margin-bottom: 15px;"
            )

    def update_light1(self, value):
        self.node.light1_val = value
        self.light1_label.setText(f"Front Light Intensity: {value}")

    def update_light2(self, value):
        self.node.light2_val = value
        self.light2_label.setText(f"Bottom Light Intensity: {value}")


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

    def handle_sigint(signum, frame):
        app.quit()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        exit_code = app.exec_()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()