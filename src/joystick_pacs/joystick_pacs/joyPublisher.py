import rclpy
from rclpy.node import Node
from nemo_interfaces.msg import RovCommands
from std_msgs.msg import Bool
import pygame
from joystick_pacs.calibrationNode import JoystickCalibrationNode


class joystickReadWrapper:
    def __init__(self, controller_id):
        pygame.init()
        pygame.joystick.init()

        self.last_button_states = {}
        self.toggle_states = {}

        self.controller = pygame.joystick.Joystick(controller_id)
        if not self.controller.get_init():
            self.controller.init()

        self.pause_read = False

    def readAxis(self, axis_id):
        try:
            pygame.event.pump()
            rawAxisVal = self.controller.get_axis(axis_id)
            return rawAxisVal
        except Exception:
            return 0.0   # safe fallback so callers always get a float

    def buttonPressedOnce(self, button_id):
        """Returns True ONLY on a 0 → 1 transition."""
        pygame.event.pump()

        current_state = self.controller.get_button(button_id)
        prev_state = self.last_button_states.get(button_id, 0)

        pressed_once = (current_state == 1 and prev_state == 0)

        self.last_button_states[button_id] = current_state
        return pressed_once

    def buttonPressed(self, button_id):
        """Returns True continuously while the button is held down."""
        pygame.event.pump()
        return self.controller.get_button(button_id) == 1


# ROS 2 master node for joystick
class joyMasterNode(Node):
    def __init__(self):
        super().__init__("joystickPublisher")

        self.declare_parameter('surgeInput', [1.0, 0.019, -1.0])
        self.declare_parameter('yawInput', [0.99, -0.01, -1.00])
        self.declare_parameter('heaveInput', [0.99, 0.0, -0.99])
        self.declare_parameter('swayInput', [-1.00, 0.00, 0.99])
        self.declare_parameter('pitchInput', [1.00, 0.00, -1.00]) # Added Pitch
        self.declare_parameter('rollInput', [1.00, 0.00, -1.00])  # Added Roll

        self.declare_parameter('surgeVelVector', [-1.00, 0.00, 1.00])
        self.declare_parameter('yawVelVector', [-1.00, 0.00, 1.00])
        self.declare_parameter('heaveVelVector', [-1.00, 0.00, 1.00])
        self.declare_parameter('swayVelVector', [-1.00, 0.00, 1.00])
        self.declare_parameter('pitchVelVector', [-1.00, 0.00, 1.00]) # Added Pitch
        self.declare_parameter('rollVelVector', [-1.00, 0.00, 1.00])  # Added Roll

        self.surgeInVector = self.get_parameter('surgeInput').value
        self.yawInVector   = self.get_parameter('yawInput').value
        self.heaveInVector = self.get_parameter('heaveInput').value
        self.swayInVector  = self.get_parameter('swayInput').value
        self.pitchInVector = self.get_parameter('pitchInput').value
        self.rollInVector  = self.get_parameter('rollInput').value

        self.inputAxisVectorMap = {
            'SURGE': self.surgeInVector,
            'YAW':   self.yawInVector,
            'HEAVE': self.heaveInVector,
            'SWAY':  self.swayInVector,
            'PITCH': self.pitchInVector,
            'ROLL':  self.rollInVector,
        }

        self.surgeOutVector = self.get_parameter('surgeVelVector').value
        self.yawOutVector   = self.get_parameter('yawVelVector').value
        self.heaveOutVector = self.get_parameter('heaveVelVector').value
        self.swayOutVector  = self.get_parameter('swayVelVector').value
        self.pitchOutVector = self.get_parameter('pitchVelVector').value
        self.rollOutVector  = self.get_parameter('rollVelVector').value

        self.outputAxisVectorMap = {
            'SURGE': self.surgeOutVector,
            'YAW':   self.yawOutVector,
            'HEAVE': self.heaveOutVector,
            'SWAY':  self.swayOutVector,
            'PITCH': self.pitchOutVector,
            'ROLL':  self.rollOutVector,
        }

        self.invertStatus = {
            'SURGE': False, 'HEAVE': False, 'SWAY': False, 
            'YAW': False, 'PITCH': False, 'ROLL': False
        }
        
        # Added virtual IDs for Pitch (100) and Roll (101) to pass through mapFunc
        # NOTE: axes 1 and 3 were swapped (surge/sway were crossed on the
        # physical stick) - axis 1 is actually SWAY, axis 3 is actually SURGE.
        self.axisIdMap = {
            0: 'YAW', 1: 'SURGE', 3: 'SWAY', 4: 'HEAVE', 
       100: 'PITCH', 101: 'ROLL'
        }

        self.startValuePublisher = False

        self.commandPublisher    = self.create_publisher(RovCommands, '/nemo_auv/input_cmd', 10)
        self.depth_lock_publisher = self.create_publisher(Bool, '/nemo_auv/lock_status', 10)

        self.commandTimer      = self.create_timer(0.1, self.publish_cmd)
        self.buttonCommandTimer = self.create_timer(0.1, self.button_publisher_timer)

        self.depth_lock_status = False

        self.joyWrapper = joystickReadWrapper(0)

        self.userInput = input("Enter Y if you want to start Calibration and N if you don't: ")
        if self.userInput.strip().upper() == "Y":
            try:
                calibrationNode = JoystickCalibrationNode()
                self.startValuePublisher = True
            except Exception:
                return
        else:
            self.startValuePublisher = True

    def publish_cmd(self):
        if not self.startValuePublisher:
            return

        msg = RovCommands()
        
        msg.surge = float(self.normalizeJoystickInput(3))
        msg.yaw   = float(self.normalizeJoystickInput(0))
        msg.heave = float(self.normalizeJoystickInput(4))
        msg.sway  = -float(self.normalizeJoystickInput(1))

        # PITCH: Combine LT (Axis 2) and RT (Axis 5)
        # Pygame triggers typically go from -1.0 (unpressed) to 1.0 (pressed).
        # We normalize them to 0.0 -> 1.0 first, then subtract to get a -1.0 -> 1.0 range.
        lt_val = self.joyWrapper.readAxis(2)
        rt_val = self.joyWrapper.readAxis(5)
        lt_norm = (lt_val + 1.0) / 2.0
        rt_norm = (rt_val + 1.0) / 2.0
        raw_pitch = rt_norm - lt_norm # RT pitches Up (+), LT pitches Down (-)
        
        pitch_mapped = self.mapFunc(raw_pitch, 100) # 100 maps to PITCH
        if abs(pitch_mapped) < 0.1:
            pitch_mapped = 0.0
        msg.roll = -float(pitch_mapped)   # SWAPPED: trigger input now drives ROLL

        # ROLL: Combine LB (Button 4) and RB (Button 5)
        lb_pressed = self.joyWrapper.buttonPressed(4)
        rb_pressed = self.joyWrapper.buttonPressed(5)
        raw_roll = float(rb_pressed) - float(lb_pressed) # RB rolls Right (+), LB rolls Left (-)
        
        roll_mapped = self.mapFunc(raw_roll, 101) # 101 maps to ROLL
        msg.pitch = float(roll_mapped)   # SWAPPED: bumper input now drives PITCH (negated: was reversed)

        self.commandPublisher.publish(msg)
        self.get_logger().info(
            f"S={msg.surge:.2f}, SW={msg.sway:.2f}, H={msg.heave:.2f}, Y={msg.yaw:.2f}, P={msg.pitch:.2f}, R={msg.roll:.2f}"
        )

    def mapFunc(self, mappingVar, axis_id):
        axis_name               = self.axisIdMap.get(axis_id)
        axis_inversion_status   = self.invertStatus.get(axis_name)
        input_calibrated_vector = self.inputAxisVectorMap.get(axis_name)
        output_velocity_vector  = self.outputAxisVectorMap.get(axis_name)

        if axis_inversion_status is True:
            input_calibrated_vector = input_calibrated_vector[::-1]
            mappingVar = mappingVar * -1.00

        mean_in  = input_calibrated_vector[1]
        mean_out = output_velocity_vector[1]
        max_in   = input_calibrated_vector[2]
        max_out  = output_velocity_vector[2]
        min_in   = input_calibrated_vector[0]
        min_out  = output_velocity_vector[0]

        if mappingVar < mean_in:
            mappedVal = ((mappingVar - mean_in) / (min_in - mean_in)) * (min_out - mean_out)
        else:
            mappedVal = ((mappingVar - mean_in) / (max_in - mean_in)) * (max_out - mean_out)

        return mappedVal

    def normalizeJoystickInput(self, axis_id):
        rawAxisVal = self.joyWrapper.readAxis(axis_id)
        if abs(rawAxisVal) < 0.1:
            rawAxisVal = 0.0

        normalizedAxisVal = self.mapFunc(rawAxisVal, axis_id)
        if abs(normalizedAxisVal) < 0.1:
            normalizedAxisVal = 0.0

        return normalizedAxisVal

    def button_publisher_timer(self):
        if self.joyWrapper.buttonPressedOnce(3):
            self.depth_lock_status = not self.depth_lock_status
            msg = Bool()
            print(f"Depth Lock Status is {self.depth_lock_status}")
            msg.data = self.depth_lock_status
            self.depth_lock_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = joyMasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()