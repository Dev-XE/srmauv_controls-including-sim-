import rclpy
import pygame
import numpy as np
from rclpy.node import Node
from nemo_interfaces.msg import RovCommands
import serial
import time
from std_msgs.msg import Bool

START_BIT = 0xAA
STOP_BIT = 0x55

class joystickReadWrapper:
    def __init__(self,controller_id):
        pygame.init()
        pygame.joystick.init()

        self.last_button_states = {}
        self.toggle_states = {}

        
        self.controller = pygame.joystick.Joystick(controller_id)
        if not self.controller.get_init():
            self.controller.init()

        self.pause_read = False

        

    def readAxis(self,axis_id):
        rawAxisval = None
        try:
            pygame.event.pump()
            rawAxisval = self.controller.get_axis(axis_id)
            return rawAxisval
        except self.pause_read:
            pass


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
            RovCommands, '/input_cmd', self.cmd_callback, 10
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
        
    
    def roll_pid_controller(self, current_angle, current_vel):
        """
        current_angle : roll angle (rad)
        current_vel   : roll rate (rad/s)
        returns       : roll control command (clamped)
        """

        # -------- Outer loop: angle → desired rate --------
        angle_error = -current_angle

        desired_rate = (
            self.roll_angle_kp * angle_error
            - self.roll_angle_kd * current_vel
        )

        # Limit desired angular rate
        desired_rate = max(
            min(desired_rate, self.max_roll_rate),
            -self.max_roll_rate
        )

        # -------- Inner loop: rate PID --------
        rate_error = desired_rate - current_vel

        # Integrator
        self.roll_rate_integral += rate_error * self.dt
        self.roll_rate_integral = max(
            min(self.roll_rate_integral, self.roll_i_limit),
            -self.roll_i_limit
        )

        control = (
            self.roll_kp * rate_error +
            self.roll_ki * self.roll_rate_integral +
            self.roll_kd * (rate_error - self.prev_roll_rate_error) / self.dt
        )

        self.prev_roll_rate_error = rate_error

        # -------- Output saturation --------
        control = max(
            min(control, self.max_roll_output),
            -self.max_roll_output
        )

        return control
    

    def pitch_pid_controller(self, current_angle, current_vel):
        """
        current_angle : pitch angle (rad)
        current_vel   : pitch rate (rad/s)
        """

        angle_error = -current_angle

        desired_rate = (
            self.pitch_angle_kp * angle_error
            - self.pitch_angle_kd * current_vel
        )

        desired_rate = max(
            min(desired_rate, self.max_pitch_rate),
            -self.max_pitch_rate
        )

        rate_error = desired_rate - current_vel

        self.pitch_rate_integral += rate_error * self.dt
        self.pitch_rate_integral = max(
            min(self.pitch_rate_integral, self.pitch_i_limit),
            -self.pitch_i_limit
        )

        control = (
            self.pitch_kp * rate_error +
            self.pitch_ki * self.pitch_rate_integral +
            self.pitch_kd * (rate_error - self.prev_pitch_rate_error) / self.dt
        )

        self.prev_pitch_rate_error = rate_error

        control = max(
            min(control, self.max_pitch_output),
            -self.max_pitch_output
        )

        return control

    def depth_pid_controller(self,desired_depth,current_depth,depth_rate):
        depth_error = desired_depth - current_depth
        dept_rate_correction = self.depth_kp * depth_error + self.depth_kd * depth_error

        
        


    def allocate_thrusters(self,msg_values):  
        allocatorArray = np.array([
            [0.707, 0.707, -0.707, -0.707, 0, 0, 0, 0],
            [-0.707, 0.707, -0.707, 0.707, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [-0.293, +0.293, +0.293, -0.293, 0, 0, 0, 0],
        ])
        #pseudo Inverse
        Binv = np.linalg.pinv(np.asmatrix(allocatorArray))
        #gain matrix 
        gain_array = np.array([
            [0.5, 0, 0, 0, 0, 0],
            [0, 0.5, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0.5],
        ])
        gain_matrix = np.asmatrix(gain_array)
        #allocation
        thrust_list = Binv * (gain_matrix * msg_values)
        for i in range(0,8):
            if thrust_list[i] >= self.max_thrust:
                thrust_list[i] = self.max_thrust
            elif thrust_list[i] <= self.min_thrust:
                thrust_list[i] = self.min_thrust
             
        return np.asarray(thrust_list)
    

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