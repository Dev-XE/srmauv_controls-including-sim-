#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16MultiArray
from sensor_msgs.msg import Joy

# ============================================================
# Joystick mapping for MANIPULATOR ONLY
# Change these if the Auto-Discovery log tells you different numbers!
# ============================================================
BTN_GRIPPER_OPEN = 2    # X button usually
BTN_GRIPPER_CLOSE = 1   # B button usually

AXIS_DPAD_L_R = 6       # Left (+1.0) / Right (-1.0)
AXIS_DPAD_U_D = 7       # Up (+1.0) / Down (-1.0)

ANGLE_STEP = 5

# ============================================================
# Servo limits (in degrees)
# ============================================================
ROLL_MIN = 0
ROLL_MAX = 200
PITCH_MIN = 0
PITCH_MAX = 90
GRIPPER_MIN = 140
GRIPPER_MAX = 175

ROLL_HOME = 90
PITCH_HOME = 0
GRIPPER_HOME = 175

class ManipulatorPublisher(Node):
    def __init__(self):
        super().__init__('manipulator_publisher')

        self.manip_pub = self.create_publisher(Int16MultiArray, 'nemo_auv/manipulator', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Subscribe to the new manipulator arm topic from the GUI
        self.arm_sub = self.create_subscription(Bool, '/nemo_auv/manipulator_arm', self.arm_callback, 10)

        self.roll = ROLL_HOME
        self.pitch = PITCH_HOME
        self.gripper = GRIPPER_HOME

        self.current_joy = None
        self.is_armed = False

        self.get_logger().info("Manipulator publisher started. [DISARMED BY DEFAULT]")
        self.get_logger().info("D-PAD controls roll/pitch. X opens gripper, B closes gripper.")
        self.get_logger().info("Waiting for /joy data and ARM signal...")

        self.control_timer = self.create_timer(0.1, self.control_loop)

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def angle_to_pwm(self, angle):
        return int(500 + (angle / 180.0) * 2000)

    def arm_callback(self, msg: Bool):
        self.is_armed = msg.data
        state_str = "ARMED" if self.is_armed else "DISARMED"
        self.get_logger().info(f"Manipulator is now {state_str}")

    def publish_angles(self):
        msg = Int16MultiArray()
        msg.data = [
            self.angle_to_pwm(self.gripper),
            self.angle_to_pwm(self.roll),
            self.angle_to_pwm(self.pitch)
        ]
        self.manip_pub.publish(msg)

        self.get_logger().info(
            f"Angles -> Roll:{self.roll}°, Pitch:{self.pitch}°, Grip:{self.gripper}° | PWM: {msg.data}"
        )

    def joy_callback(self, msg):
        self.current_joy = msg

    def control_loop(self):
        # Ignore joystick input if the manipulator is disarmed
        if not self.is_armed:
            return

        if self.current_joy is None:
            return

        changed = False
        msg = self.current_joy

        # Auto-Discovery for debugging D-Pad axes
        active_axes = [i for i, val in enumerate(msg.axes) if abs(val) > 0.5]
        active_btns = [i for i, val in enumerate(msg.buttons) if val == 1]
        
        # --- D-PAD CONTROL ---
        if len(msg.axes) > max(AXIS_DPAD_L_R, AXIS_DPAD_U_D):
            dpad_l_r = msg.axes[AXIS_DPAD_L_R]
            dpad_u_d = msg.axes[AXIS_DPAD_U_D]

            if dpad_u_d > 0.5:
                self.pitch += ANGLE_STEP
                changed = True
            elif dpad_u_d < -0.5:
                self.pitch -= ANGLE_STEP
                changed = True

            if dpad_l_r > 0.5:
                self.roll += ANGLE_STEP
                changed = True
            elif dpad_l_r < -0.5:
                self.roll -= ANGLE_STEP
                changed = True

        # --- GRIPPER CONTROL ---
        if len(msg.buttons) > max(BTN_GRIPPER_OPEN, BTN_GRIPPER_CLOSE):
            if msg.buttons[BTN_GRIPPER_OPEN] == 1:
                self.gripper -= ANGLE_STEP
                changed = True
            if msg.buttons[BTN_GRIPPER_CLOSE] == 1:
                self.gripper += ANGLE_STEP
                changed = True

        # Print debug info if buttons/axes are pressed but nothing changed
        if not changed and (active_axes or active_btns):
             self.get_logger().debug(f"Input detected but not mapped -> Axes: {active_axes}, Buttons: {active_btns}")

        if changed:
            self.roll = self.clamp(self.roll, ROLL_MIN, ROLL_MAX)
            self.pitch = self.clamp(self.pitch, PITCH_MIN, PITCH_MAX)
            self.gripper = self.clamp(self.gripper, GRIPPER_MIN, GRIPPER_MAX)
            self.publish_angles()

def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()