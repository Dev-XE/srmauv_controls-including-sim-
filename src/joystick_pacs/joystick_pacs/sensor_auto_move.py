import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
import threading
import math

class SensorAutoMoveNode(Node):
    def __init__(self):
        super().__init__('sensor_auto_move_node')
        
        # Publishers and Subscribers
        self.publisher = self.create_publisher(Twist, '/srmauv/cmd_vel', 10)
        self.sonar_sub = self.create_subscription(LaserScan, '/srmauv/ping360/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/srmauv/imu', self.imu_callback, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State variables
        self.state = 'IDLE'
        self.start_rear_dist = 0.0
        self.current_rear_dist = 0.0
        self.target_yaw = 0.0
        self.current_yaw = 0.0
        
        self.has_sonar = False
        self.has_imu = False
        
        # Proportional gain for heading correction
        self.kp_yaw = 1.2 

    def scan_callback(self, msg):
        # Average the rays pointing directly backward (indices at the start and end of the array)
        window_size = 5
        rear_rays = msg.ranges[:window_size] + msg.ranges[-window_size:]
        
        valid_rays = [r for r in rear_rays if not math.isinf(r) and 0.4 < r < 15.0]
        
        if valid_rays:
            self.current_rear_dist = sum(valid_rays) / len(valid_rays)
            self.has_sonar = True

    def imu_callback(self, msg):
        # Convert IMU quaternion to Yaw (Euler Z-axis angle)
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_imu = True

    def control_loop(self):
        if not self.has_sonar or not self.has_imu or self.state == 'IDLE':
            return
            
        msg = Twist()
        
        # Calculate heading error and normalize between -PI and +PI
        yaw_error = self.target_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        
        # Generate corrective rotational velocity
        correction_z = self.kp_yaw * yaw_error
        
        if self.state == 'FORWARD':
            if self.current_rear_dist < (self.start_rear_dist + 1.0):
                msg.linear.y = -0.6  # Changed to negative for forward surge
                msg.angular.z = correction_z
            else:
                self.get_logger().info(f'Reached 1m target. Stopping and reversing.')
                self.state = 'BACKWARD'
                msg.linear.y = 0.0
                msg.angular.z = 0.0
                
        elif self.state == 'BACKWARD':
            if self.current_rear_dist > self.start_rear_dist:
                msg.linear.y = 0.6  # Changed to positive for reverse surge
                msg.angular.z = correction_z
            else:
                self.get_logger().info(f'Returned to start.')
                self.state = 'IDLE'
                msg.linear.y = 0.0
                msg.angular.z = 0.0
        
        self.publisher.publish(msg)

    def start_sequence(self):
        if not self.has_sonar or not self.has_imu:
            print("\nError: Missing sensor data. Check Ping360 and IMU topics.")
            return
            
        self.start_rear_dist = self.current_rear_dist
        self.target_yaw = self.current_yaw
        self.state = 'FORWARD'
        print(f"\nSequence Initiated.")
        print(f"Target Yaw locked at: {self.target_yaw:.2f} rad")
        print(f"Start wall distance locked at: {self.start_rear_dist:.2f} m")

    def stop_sequence(self):
        self.state = 'IDLE'
        msg = Twist()
        self.publisher.publish(msg)
        print("\nSequence Halted: ROV stopped.")

def terminal_input(node):
    while rclpy.ok():
        cmd = input("\n[Terminal Control] Type 'start' to run, 'stop' to halt, 'exit' to quit: \n")
        
        if cmd.strip().lower() == 'start':
            node.start_sequence()
        elif cmd.strip().lower() == 'stop':
            node.stop_sequence()
        elif cmd.strip().lower() == 'exit':
            node.stop_sequence()
            print("Exiting...")
            break

def main(args=None):
    rclpy.init(args=args)
    node = SensorAutoMoveNode()
    
    input_thread = threading.Thread(target=terminal_input, args=(node,), daemon=True)
    input_thread.start()
    
    try:
        while rclpy.ok() and input_thread.is_alive():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
