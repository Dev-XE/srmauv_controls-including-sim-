import rclpy
from rclpy.node import Node
from nemo_interfaces.msg import RovCommands
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class CommandTranslator(Node):
    def __init__(self):
        super().__init__('command_translator')
        
        # Define the exact ceiling limit from your odometry output
        self.surface_z = 1.98
        self.current_z = 0.0
        self.odom_received = False
        
        # Subscribe to Gazebo odometry to track depth
        self.odom_sub = self.create_subscription(
            Odometry,
            '/srmauv/odom',
            self.odom_callback,
            10)

        # Subscribe to your joystick script
        self.subscription = self.create_subscription(
            RovCommands,
            '/nemo_auv/input_cmd',
            self.listener_callback,
            10)
            
        # Publish to the Gazebo simulator
        self.publisher = self.create_publisher(
            Twist, 
            '/srmauv/cmd_vel', 
            10)

    def odom_callback(self, msg):
        self.current_z = msg.pose.pose.position.z
        self.odom_received = True

    def listener_callback(self, msg):
        twist = Twist()
        
        # Map linear velocities
        twist.linear.x = msg.surge
        twist.linear.y = msg.sway
        
        heave = msg.heave
        
        # Apply the ceiling limit to heave
        if self.odom_received:
            # Block upward thrust if Z is greater than or equal to the surface limit
            if self.current_z >= self.surface_z and heave > 0.0:
                heave = 0.0  
                self.get_logger().info("Ceiling limit active! Upward thrust blocked.", throttle_duration_sec=2.0)

        twist.linear.z = heave
        
        # Map angular velocities
        twist.angular.x = msg.roll
        twist.angular.y = msg.pitch
        twist.angular.z = msg.yaw
        
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    translator = CommandTranslator()
    try:
        rclpy.spin(translator)
    except KeyboardInterrupt:
        pass
    finally:
        translator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
