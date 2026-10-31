import rclpy
from rclpy.node import Node
from nemo_interfaces.msg import RovCommands
from std_msgs.msg import Float64

class ThrusterMixerNode(Node):
    def __init__(self):
        super().__init__('thruster_mixer')
        
        # Subscribe to your unmodified joyPublisher script
        self.subscription = self.create_subscription(
            RovCommands,
            '/nemo_auv/input_cmd',
            self.listener_callback,
            10)
            
        # Create publishers for the 8 thrusters 
        self.pub_hz_fl = self.create_publisher(Float64, '/srmauv/thrusters/horiz_front_left', 10)
        self.pub_hz_fr = self.create_publisher(Float64, '/srmauv/thrusters/horiz_front_right', 10)
        self.pub_hz_bl = self.create_publisher(Float64, '/srmauv/thrusters/horiz_back_left', 10)
        self.pub_hz_br = self.create_publisher(Float64, '/srmauv/thrusters/horiz_back_right', 10)
        
        self.pub_vt_fl = self.create_publisher(Float64, '/srmauv/thrusters/vert_front_left', 10)
        self.pub_vt_fr = self.create_publisher(Float64, '/srmauv/thrusters/vert_front_right', 10)
        self.pub_vt_bl = self.create_publisher(Float64, '/srmauv/thrusters/vert_back_left', 10)
        self.pub_vt_br = self.create_publisher(Float64, '/srmauv/thrusters/vert_back_right', 10)

    def listener_callback(self, msg):
        # Extract 6 Degrees of Freedom
        s = msg.surge
        sw = msg.sway
        h = msg.heave
        r = msg.roll
        p = msg.pitch
        y = msg.yaw

        # Calculate Horizontal Motor Outputs (45-degree vectoring)
        hz_fl = s + sw + y
        hz_fr = s - sw - y
        hz_bl = s - sw + y
        hz_br = s + sw - y
        
        # Calculate Vertical Motor Outputs
        vt_fl = h + p - r
        vt_fr = h + p + r
        vt_bl = h - p - r
        vt_br = h - p + r
        
        # Publish the specific commands
        self.pub_hz_fl.publish(Float64(data=float(hz_fl)))
        self.pub_hz_fr.publish(Float64(data=float(hz_fr)))
        self.pub_hz_bl.publish(Float64(data=float(hz_bl)))
        self.pub_hz_br.publish(Float64(data=float(hz_br)))
        
        self.pub_vt_fl.publish(Float64(data=float(vt_fl)))
        self.pub_vt_fr.publish(Float64(data=float(vt_fr)))
        self.pub_vt_bl.publish(Float64(data=float(vt_bl)))
        self.pub_vt_br.publish(Float64(data=float(vt_br)))

def main(args=None):
    rclpy.init(args=args)
    mixer = ThrusterMixerNode()
    try:
        rclpy.spin(mixer)
    except KeyboardInterrupt:
        pass
    finally:
        mixer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
