import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import socket

class UDPBridge(Node):
    def __init__(self):
        super().__init__('udp_bridge')
        # UPDATED: Matches your verified ESP IP from the ping test
        self.esp_ip = "10.125.204.101"  
        self.udp_port = 4210
        
        # Create socket with a 0.1s timeout so ROS doesn't hang forever
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1) 

        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/target_angles',
            self.callback,
            10
        )
        self.get_logger().info(f'🚀 UDP Bridge Online. Targeting Robot at {self.esp_ip}')

    def callback(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn(f"Expected 4 angles, got {len(msg.data)}")
            return

        # Format: "base,shoulder,elbow,gripper"
        data_str = ",".join(map(str, msg.data))
        
        try:
            # 1. Send command
            self.sock.sendto(data_str.encode(), (self.esp_ip, self.udp_port))
            
            # 2. Sensing: Wait for the ACK we verified in terminal
            data, addr = self.sock.recvfrom(1024)
            if data.decode() == "ACK":
                self.get_logger().info(f"✅ Robot Synced: {data_str}")
            
        except socket.timeout:
            # This happens if the 8V battery sags or Wi-Fi jitters
            self.get_logger().error("⚠️ SENSING TIMEOUT: Robot missed a frame!")
        except Exception as e:
            self.get_logger().error(f"❌ UDP Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UDPBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down UDP Bridge...')
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()