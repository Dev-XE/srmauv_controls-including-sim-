#!/usr/bin/env python3
import rclpy
import pygame
from rclpy.node import Node
from std_msgs.msg import Bool
import time


class joystickReadWrapper:
    def __init__(self, controller_id):
        pygame.init()
        pygame.joystick.init()

        self.controller = pygame.joystick.Joystick(controller_id)
        if not self.controller.get_init():
            self.controller.init()

    def readButton(self, button_id):
        """Reads the button state (True when pressed)."""
        try:
            pygame.event.pump()
            return self.controller.get_button(button_id) == 1
        except Exception as e:
            print(f"⚠️ Joystick read error: {e}")
            return False


class ArmStatusPublisher(Node):
    def __init__(self):
        super().__init__('arm_status_publisher')

        # Publisher for /arm_status topic
        self.publisher_ = self.create_publisher(Bool, '/arm_status', 10)

        # Initialize joystick
        self.joystick = joystickReadWrapper(0)

        # Timing
        self.last_state = False
        self.last_publish_time = time.time()

        # Check joystick state every 50ms
        self.timer = self.create_timer(0.05, self.arm_callback)

    def arm_callback(self):
        """Reads button 0 and publishes if changed."""
        state = self.joystick.readButton(0)
        now = time.time()

        # Publish only if state changes or every 0.5s (heartbeat)
        if state != self.last_state or (now - self.last_publish_time > 0.5):
            msg = Bool()
            msg.data = bool(state)
            self.publisher_.publish(msg)
            self.last_state = state
            self.last_publish_time = now

            status_str = "ARM PRESSED ✅" if msg.data else "ARM RELEASED ❌"
            self.get_logger().info(f"Button 0 → {status_str}")


def main(args=None):
    rclpy.init(args=args)
    node = ArmStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
