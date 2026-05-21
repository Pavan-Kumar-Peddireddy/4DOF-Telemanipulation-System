import os
import socket
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import warnings

# --- FIX TERMINAL ERRORS ---
# Silences the Qt Font and Protobuf warnings from your logs
os.environ["QT_QPA_PLATFORM"] = "xcb" 
warnings.filterwarnings("ignore", category=UserWarning)

from mediapipe.python.solutions import hands as mp_hands
from mediapipe.python.solutions import drawing_utils as mp_draw
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# --- PHYSICAL SETTINGS (From your working example) ---
L1, L2 = 9.3, 11.0
SMOOTHING = 0.08  
MAX_SPEED = 0.15 

class MovementSequencer(Node):
    def __init__(self):
        super().__init__('movement_sequencer')
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        # Real-time ESP32 Control
        self.esp_ip, self.udp_port = "10.125.204.101", 4210
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.005)

        self.hands = mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5)
        self.cap = cv2.VideoCapture(0)

        self.current_pos = [0.0, 0.0, 0.0, 0.0]
        self.target_pos = [0.0, 0.0, 0.0, 0.0]
        self.history = [0] * 5 

        self.active_task = "MANUAL" 
        self.task_step = 0
        self.task_wait = 0
        
        self.timer = self.create_timer(0.05, self.run_loop)
        self.get_logger().info("✅ Sequencer Synced with Working Example Logic")

    def run_loop(self):
        ret, frame = self.cap.read()
        if not ret: return
        frame = cv2.flip(frame, 1)
        res = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        # 1. FINGER LOGIC (Exactly like your working code)
        fingers = 0
        if res.multi_hand_landmarks:
            for lms in res.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, lms, mp_hands.HAND_CONNECTIONS)
                for tip in [8, 12, 16, 20]:
                    if lms.landmark[tip].y < lms.landmark[tip - 2].y: fingers += 1
                if lms.landmark[4].x > lms.landmark[3].x: fingers += 1

        self.history.pop(0); self.history.append(fingers)
        smoothed = max(set(self.history), key=self.history.count)

        # 2. STATE SELECTION
        if self.active_task == "MANUAL":
            if smoothed == 1:
                self.active_task = "EXTENDED_PICK"
                self.task_step = 0; self.task_wait = 0
            elif smoothed == 5:
                self.target_pos = [0.0, 0.0, 0.0, 0.0]

        # 3. FORWARD REACH LOGIC (Fixes "Picking at foot")
        if self.active_task == "EXTENDED_PICK":
            # Joint angles forced to reach > 10cm forward
            # [q1, q2 (Shoulder), q3 (Elbow), q4 (Gripper)]
            steps = [
                [0.0, 0.5, -0.2, 0.0],   # Step 0: Lift and Move Out
                [0.0, 1.0, -0.6, 0.0],   # Step 1: Reach FAR Forward (14cm+)
                [0.0, -0.3, -1.3, 0.0],  # Step 2: Drop Down
                [0.0, -0.3, -1.3, 1.4],  # Step 3: GRIPPER CLOSE (Snap Movement)
                [0.0, 0.6, -0.4, 1.4]    # Step 4: Lift Up
            ]
            self.target_pos = steps[self.task_step]
            self.task_wait += 1
            
            # Ensure gripper has time to close in simulation
            limit = 150 if self.task_step == 3 else 70
            if self.task_wait > limit:
                self.task_step += 1; self.task_wait = 0
                if self.task_step >= len(steps): self.active_task = "MANUAL"

        # 4. INTERPOLATION & SYNC (Like working example)
        for i in range(4):
            diff = self.target_pos[i] - self.current_pos[i]
            self.current_pos[i] += np.clip(diff * SMOOTHING, -MAX_SPEED, MAX_SPEED)

        angles = [int(np.degrees(p) + 90) for p in self.current_pos]
        data_str = ",".join(map(str, angles))

        # ROS 2 Gazebo Sync
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        pt = JointTrajectoryPoint(positions=list(self.current_pos), time_from_start=Duration(nanosec=50000000))
        msg.points.append(pt)
        self.publisher_.publish(msg)

        # ESP32 Sync
        try: self.sock.sendto(data_str.encode(), (self.esp_ip, self.udp_port))
        except: pass

        # 5. UI (Fixed Font)
        cv2.putText(frame, f"Fingers: {smoothed}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Bot Sequencer", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = MovementSequencer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        # EXACT cleanup from your working example
        node.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()