import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')
        
        # Topic configurations
        self.subscription = self.create_subscription(
            Imu,
            '/srmauv/imu',
            self.imu_callback,
            10
        )
        self.publisher = self.create_publisher(
            Imu,
            '/srmauv/imu/calibrated',
            10
        )
        
        # Calibration parameters
        self.calibration_samples = 100
        self.current_samples = 0
        self.is_calibrated = False
        
        # Bias storage
        self.gyro_bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.accel_bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        self.get_logger().info('Starting IMU calibration. Keep the vehicle stationary.')

    def imu_callback(self, msg):
        if not self.is_calibrated:
            self.collect_calibration_data(msg)
        else:
            self.publish_calibrated_data(msg)

    def collect_calibration_data(self, msg):
        # Accumulate gyro data
        self.gyro_bias['x'] += msg.angular_velocity.x
        self.gyro_bias['y'] += msg.angular_velocity.y
        self.gyro_bias['z'] += msg.angular_velocity.z
        
        # Accumulate accel data
        self.accel_bias['x'] += msg.linear_acceleration.x
        self.accel_bias['y'] += msg.linear_acceleration.y
        self.accel_bias['z'] += msg.linear_acceleration.z
        
        self.current_samples += 1
        
        if self.current_samples >= self.calibration_samples:
            self.finalize_calibration()

    def finalize_calibration(self):
        # Calculate averages
        for axis in self.gyro_bias:
            self.gyro_bias[axis] /= self.calibration_samples
            
        for axis in self.accel_bias:
            self.accel_bias[axis] /= self.calibration_samples
            
        # Account for gravity (9.81 m/s^2) on the Z-axis
        self.accel_bias['z'] -= 9.81
        
        self.is_calibrated = True
        
        self.get_logger().info('Calibration complete.')

    def publish_calibrated_data(self, msg):
        calibrated_msg = msg
        
        # Subtract calculated biases
        calibrated_msg.angular_velocity.x -= self.gyro_bias['x']
        calibrated_msg.angular_velocity.y -= self.gyro_bias['y']
        calibrated_msg.angular_velocity.z -= self.gyro_bias['z']
        
        calibrated_msg.linear_acceleration.x -= self.accel_bias['x']
        calibrated_msg.linear_acceleration.y -= self.accel_bias['y']
        calibrated_msg.linear_acceleration.z -= self.accel_bias['z']
        
        self.publisher.publish(calibrated_msg)
        
        # Print raw values to terminal
        o = calibrated_msg.orientation
        a = calibrated_msg.angular_velocity
        l = calibrated_msg.linear_acceleration
        
        self.get_logger().info(
            f'Orientation (x,y,z,w): [{o.x:.4f}, {o.y:.4f}, {o.z:.4f}, {o.w:.4f}] | '
            f'Angular Vel (x,y,z): [{a.x:.4f}, {a.y:.4f}, {a.z:.4f}] | '
            f'Linear Accel (x,y,z): [{l.x:.4f}, {l.y:.4f}, {l.z:.4f}]'
        )

def main(args=None):
    rclpy.init(args=args)
    node = IMUCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()