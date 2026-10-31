#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MinimalDepthSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_depth_subscriber')
        
        # Subscribe to the depth topic published by the Teensy
        self.subscription = self.create_subscription(
            Float32,
            '/nemo_auv/depth',
            self.depth_callback,
            10
        )
        self.get_logger().info("Terminal Depth Monitor Started. Waiting for data...")

    def depth_callback(self, msg):
        # Print the raw depth value directly to the terminal
        self.get_logger().info(f"Current Depth: {msg.data:.3f} meters")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalDepthSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()