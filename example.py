import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class GateDetector(Node):
    def __init__(self):
        super().__init__('gate_detector')
        
        # Subscribe to your raw camera feed
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        # Publish the (x, y) pixel coordinates of the gate center
        self.publisher_ = self.create_publisher(Point, '/gate_center', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # Convert to HSV color space for better color isolation
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV limits. 
        # These values are tuned for a bright orange racing gate. 
        # You will need to adjust these for your specific gate color.
        lower_orange = np.array([5, 150, 150])
        upper_orange = np.array([15, 255, 255])

        # Create a binary mask isolating the target color
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Find continuous shapes (contours) in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Assume the largest contour by area is the gate
            largest_contour = max(contours, key=cv2.contourArea)

            # Calculate spatial moments to find the geometric center
            M = cv2.moments(largest_contour)
            
            # Prevent division by zero if the contour area is exactly 0
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Publish the center coordinates
                center_msg = Point()
                center_msg.x = float(cx)
                center_msg.y = float(cy)
                center_msg.z = 0.0  # Z is unused for 2D image coordinates
                self.publisher_.publish(center_msg)

                self.get_logger().debug(f"Gate center at: X={cx}, Y={cy}")
            else:
                self.get_logger().debug("Gate color detected, but area is zero.")
        else:
            self.get_logger().debug("No gate detected.")

def main(args=None):
    rclpy.init(args=args)
    node = GateDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
