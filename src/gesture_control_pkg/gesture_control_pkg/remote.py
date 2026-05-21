import os
import socket
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import warnings

# --- SILENCE TERMINAL ERRORS ---
os.environ["QT_QPA_PLATFORM"] = "xcb" 
warnings.filterwarnings("ignore", category=UserWarning)

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# --- CONFIGURATION ---
STEP_SIZE = 0.05 
SMOOTHING = 0.15 
G_OPEN, G_CLOSE = 0.0, 1.2 # Radian limits for gripper

class KeyboardRemoteNode(Node):
    def __init__(self):
        super().__init__('kb_remote_node')
        # Ensure this matches your controller name in ros2_control
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        self.esp_ip, self.udp_port = "10.125.204.101", 4210
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.005)

        # [Base, Shoulder, Elbow, Gripper]
        self.current_pos = [0.0, 0.0, 0.0, 0.0]
        self.target_pos = [0.0, 0.0, 0.0, 0.0]
        
        self.recorded_path = [] 
        self.mode = "MANUAL" 
        self.play_index = 0

        cv2.namedWindow("Cyborg 15 Keyboard Remote")
        self.create_timer(0.05, self.run_loop)
        self.get_logger().info("✅ Remote Fixed: Degrees Capped & Sim Sync Active")

    def run_loop(self):
        status_img = np.zeros((450, 650, 3), dtype=np.uint8)
        key = cv2.waitKey(1) & 0xFF

        # --- SYSTEM CONTROLS ---
        if key == ord('r'):
            self.mode = "RECORDING" if self.mode != "RECORDING" else "MANUAL"
        elif key == ord('p'):
            if self.recorded_path: self.mode = "PLAYBACK" if self.mode != "PLAYBACK" else "MANUAL"
            self.play_index = 0
        elif key == ord('c'):
            self.recorded_path = []; self.target_pos = [0.0, 0.0, 0.0, 0.0]; self.mode = "MANUAL"

        # --- MOVEMENT (Strict Clipping to prevent 600+ degrees) ---
        if self.mode != "PLAYBACK":
            if key == ord('a'): self.target_pos[0] = np.clip(self.target_pos[0] + STEP_SIZE, -1.57, 1.57)
            if key == ord('d'): self.target_pos[0] = np.clip(self.target_pos[0] - STEP_SIZE, -1.57, 1.57)
            if key == ord('w'): self.target_pos[1] = np.clip(self.target_pos[1] + STEP_SIZE, -1.2, 1.2)
            if key == ord('s'): self.target_pos[1] = np.clip(self.target_pos[1] - STEP_SIZE, -1.2, 1.2)
            if key == ord('q'): self.target_pos[2] = np.clip(self.target_pos[2] + STEP_SIZE, -1.5, 1.5)
            if key == ord('e'): self.target_pos[2] = np.clip(self.target_pos[2] - STEP_SIZE, -1.5, 1.5)
            if key == ord('z'): self.target_pos[3] = G_OPEN
            if key == ord('x'): self.target_pos[3] = G_CLOSE

        # --- RECORDING/PLAYBACK LOGIC ---
        if self.mode == "RECORDING":
            self.recorded_path.append(list(self.target_pos))
        elif self.mode == "PLAYBACK" and self.recorded_path:
            self.target_pos = self.recorded_path[self.play_index]
            self.play_index = (self.play_index + 1) % len(self.recorded_path)

        # Interpolation (Smooth following)
        for i in range(4):
            self.current_pos[i] += (self.target_pos[i] - self.current_pos[i]) * SMOOTHING

        # --- DATA SYNC (SAFE MATH) ---
        # Clipping raw radians again before degree conversion to stop "600 degree" bug
        safe_rads = [np.clip(r, -1.57, 1.57) for r in self.current_pos]
        angles = [int(np.degrees(r) + 90) for r in safe_rads]
        data_str = ",".join(map(str, angles))
        
        # --- PUBLISH TO SIMULATION (Gazebo Fix) ---
        msg = JointTrajectory()
        # Verify these names match your URDF exactly!
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        pt = JointTrajectoryPoint()
        pt.positions = [float(r) for r in safe_rads]
        # Gazebo needs a clear time_from_start to execute
        pt.time_from_start = Duration(sec=0, nanosec=50000000) 
        msg.points.append(pt)
        self.publisher_.publish(msg)
        
        # --- SEND TO PHYSICAL ESP32 ---
        try: self.sock.sendto(data_str.encode(), (self.esp_ip, self.udp_port))
        except: pass

        # --- UI DISPLAY ---
        mode_color = (0, 0, 255) if self.mode == "RECORDING" else (0, 255, 0)
        cv2.putText(status_img, f"MODE: {self.mode}", (20, 60), cv2.FONT_HERSHEY_DUPLEX, 1.2, mode_color, 2)
        cv2.putText(status_img, f"SERVO DEGS: {angles}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        cv2.putText(status_img, f"RADIAN VALS: {[round(r,2) for r in safe_rads]}", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        cv2.line(status_img, (20, 210), (630, 210), (100, 100, 100), 1)
        cv2.putText(status_img, "WASD: Base/Shoulder | QE: Elbow | ZX: Grip", (30, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
        cv2.putText(status_img, "R: Record | P: Playback | C: Clear", (30, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 128, 0), 2)
        
        cv2.imshow("Cyborg 15 Keyboard Remote", status_img)

def main():
    rclpy.init()
    node = KeyboardRemoteNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: cv2.destroyAllWindows(); rclpy.shutdown()

if __name__ == '__main__':
    main()