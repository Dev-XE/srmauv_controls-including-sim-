#!/usr/bin/env python3
import sys
import signal
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nemo_interfaces.msg import RovCommands   # ← was missing; this is what joyPublisher sends

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QPushButton,
    QVBoxLayout,
)
from PyQt5.QtCore import QTimer


class ArmGuiNode(Node):
    def __init__(self):
        super().__init__('arm_gui_node')

        # Thruster arm publisher
        self.arm_pub = self.create_publisher(Bool, '/nemo_auv/arm', 10)

        # Manipulator arm publisher (untouched)
        self.manip_arm_pub = self.create_publisher(Bool, '/nemo_auv/manipulator_arm', 10)

        # Cmd_vel publisher — forwards joystick commands to the Teensy
        # Cmd_vel replaced with direct Wrench publisher for Gazebo physics
        self.wrench_pub = self.create_publisher(Wrench, '/srmauv/wrench', 10)

        # FIX: subscribe to RovCommands (what joyPublisher actually publishes)
        # instead of the Twist /nemo_auv/cmd_vel_raw topic that nobody was publishing.
        self.input_cmd_sub = self.create_subscription(
            RovCommands,
            '/nemo_auv/input_cmd',
            self.input_cmd_callback,
            10
        )

        # Internal states
        self.thruster_armed_state = False
        self.manip_armed_state = False

        # Heartbeat at 2 Hz so the Teensy never misses an arm state
        self.heartbeat_timer = self.create_timer(0.5, self.publish_heartbeat)

        self.last_log_time = 0.0

        self.get_logger().info('System Control GUI node started. Waiting for /nemo_auv/input_cmd...')

    def set_thruster_arm(self, state: bool):
        self.thruster_armed_state = state
        self.get_logger().info(f'>> GUI Button Clicked - Thrusters state set to: {state}')
        self.publish_heartbeat()  # Publish immediately on change

    def set_manip_arm(self, state: bool):
        self.manip_armed_state = state
        self.get_logger().info(f'>> GUI Button Clicked - Manipulator state set to: {state}')
        self.publish_heartbeat()  # Publish immediately on change

    def publish_heartbeat(self):
        t_msg = Bool()
        t_msg.data = self.thruster_armed_state
        self.arm_pub.publish(t_msg)

        m_msg = Bool()
        m_msg.data = self.manip_armed_state
        self.manip_arm_pub.publish(m_msg)

    def input_cmd_callback(self, msg: RovCommands):
        # Drop commands when thrusters are disarmed — don't feed Gazebo
        if not self.thruster_armed_state:
            return

        out_msg = Wrench()
        
        # Apply force/torque multipliers so a 1.0 joystick input can move the 25kg AUV
        FORCE_MULT = 80.0
        TORQUE_MULT = 30.0
        
        out_msg.force.x = float(msg.surge) * FORCE_MULT
        out_msg.force.y = float(msg.sway) * -FORCE_MULT # Inverted for standard ROS sway
        out_msg.force.z = float(msg.heave) * FORCE_MULT
        
        out_msg.torque.x = float(msg.roll) * TORQUE_MULT
        out_msg.torque.y = float(msg.pitch) * TORQUE_MULT
        out_msg.torque.z = float(msg.yaw) * TORQUE_MULT

        self.wrench_pub.publish(out_msg)

        current_time = time.time()
        if current_time - self.last_log_time > 2.0:
            self.get_logger().info(
                f"Applying Forces -> Fx: {out_msg.force.x:.1f}, "
                f"Fy: {out_msg.force.y:.1f}, Fz: {out_msg.force.z:.1f}, "
                f"Tx: {out_msg.torque.x:.1f}, Ty: {out_msg.torque.y:.1f}, Tz: {out_msg.torque.z:.1f}"
            )
            self.last_log_time = current_time

class ArmWindow(QWidget):
    def __init__(self, ros_node: ArmGuiNode):
        super().__init__()
        self.node = ros_node
        self.thruster_armed = False
        self.manip_armed = False

        self.setWindowTitle("AUV System Control")
        self.setMinimumSize(400, 300)

        # Thruster arm button
        self.thruster_button = QPushButton("THRUSTERS: DISARMED")
        self.thruster_button.setStyleSheet(
            "font-size: 24px; padding: 30px; background-color: red; color: white; margin-bottom: 10px;"
        )
        self.thruster_button.clicked.connect(self.toggle_thruster_arm)

        # Manipulator arm button (untouched)
        self.manip_button = QPushButton("MANIPULATOR: DISARMED")
        self.manip_button.setStyleSheet(
            "font-size: 24px; padding: 30px; background-color: red; color: white; margin-top: 10px;"
        )
        self.manip_button.clicked.connect(self.toggle_manip_arm)

        layout = QVBoxLayout()
        layout.addWidget(self.thruster_button)
        layout.addWidget(self.manip_button)
        self.setLayout(layout)

    def toggle_thruster_arm(self):
        self.thruster_armed = not self.thruster_armed
        self.node.set_thruster_arm(self.thruster_armed)

        if self.thruster_armed:
            self.thruster_button.setText("THRUSTERS: ARMED")
            self.thruster_button.setStyleSheet(
                "font-size: 24px; padding: 30px; background-color: green; color: white; margin-bottom: 10px;"
            )
        else:
            self.thruster_button.setText("THRUSTERS: DISARMED")
            self.thruster_button.setStyleSheet(
                "font-size: 24px; padding: 30px; background-color: red; color: white; margin-bottom: 10px;"
            )

    def toggle_manip_arm(self):
        self.manip_armed = not self.manip_armed
        self.node.set_manip_arm(self.manip_armed)

        if self.manip_armed:
            self.manip_button.setText("MANIPULATOR: ARMED")
            self.manip_button.setStyleSheet(
                "font-size: 24px; padding: 30px; background-color: green; color: white; margin-top: 10px;"
            )
        else:
            self.manip_button.setText("MANIPULATOR: DISARMED")
            self.manip_button.setStyleSheet(
                "font-size: 24px; padding: 30px; background-color: red; color: white; margin-top: 10px;"
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