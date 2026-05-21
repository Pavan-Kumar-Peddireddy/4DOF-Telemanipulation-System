import socket
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from mediapipe.python.solutions import hands as mp_hands
from mediapipe.python.solutions import drawing_utils as mp_draw
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import os
import warnings

os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'
warnings.filterwarnings("ignore", category=UserWarning, module='google.protobuf.symbol_database')

# ── PHYSICAL CONSTANTS (cm) ───────────────────────────────────────────────────
L1, L2, BASE_H       = 9.3, 11.0, 6.5
MAX_REACH, MIN_REACH = 18.0, 6.5
GROUND_LEVEL         = 0.3

# ── MOTION ────────────────────────────────────────────────────────────────────
SMOOTHING = 0.08
STEP_SIZE = 0.05
MAX_STEP  = 0.3

# ── GRIPPER ───────────────────────────────────────────────────────────────────
# 0 deg → pulse 150 → CLOSED  |  90 deg → pulse 450 → OPEN
# q4: 0.0 = open, GRIPPER_MAX_RAD = closed
GRIPPER_MAX_RAD = 1.57   # pi/2, 90 deg of servo travel


def gripper_to_servo_deg(q4: float) -> int:
    """Inverted mapping: q4=0 → 90 deg (open), q4=1.57 → 0 deg (closed)."""
    q4 = max(0.0, min(GRIPPER_MAX_RAD, q4))
    return int(90.0 * (1.0 - q4 / GRIPPER_MAX_RAD))


def apply_dynamic_kinematics(joints):
    q1, q2, q3, q4 = joints

    reach    = L1 * np.cos(q2) + L2 * np.cos(q2 + q3)
    z_height = BASE_H + L1 * np.sin(q2) + L2 * np.sin(q2 + q3)

    dynamic_max = MAX_REACH - max(0.0, 5.0 - z_height) * 0.6
    if reach > dynamic_max:
        q3 += 0.06
    elif reach < MIN_REACH:
        q3 -= 0.06

    if z_height < GROUND_LEVEL:
        q2 += 0.04

    return [
        np.clip(q1, -1.5,          1.5),
        np.clip(q2, -1.3,          1.4),
        np.clip(q3, -1.6,          1.6),
        np.clip(q4,  0.0, GRIPPER_MAX_RAD),
    ]


class BotController(Node):
    def __init__(self):
        super().__init__('bot_controller')
        self.publisher_ = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.esp_ip   = "10.125.204.101"
        self.udp_port = 4210
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.005)

        self.hands = mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.5,
        )
        self.cap = cv2.VideoCapture(0)

        self.current_positions = [0.0, 0.0, 0.0, 0.0]
        self.target_positions  = [0.0, 0.0, 0.0, 0.0]

        # Separate histories for fingers and thumb
        self.finger_history = [0] * 5
        self.thumb_history  = [False] * 5   # True = extended = gripper closed

        self.timer = self.create_timer(0.05, self.processing_loop)
        self.get_logger().info(f"Bot Controller Active. Target: {self.esp_ip}")

    def _detect(self, lms):
        """
        Fully decoupled detection. Returns (four_fingers, thumb_extended, thumb_is_left).

        four_fingers  : count of index/middle/ring/pinky extended (0-4)
                        tip.y < pip.y in mirrored frame = finger up
        thumb_extended: True if thumb tip is left of pinky MCP (lm17)
                        = thumb sticking out sideways = GRIPPER CLOSED
        thumb_is_left : True if thumb tip is left of index MCP (lm5)
                        = used to select LEFT vs RIGHT direction for joints

        KEY DESIGN: thumb is NOT counted in four_fingers.
        finger count = WHICH joint moves
        thumb_extended = gripper state  (independent, any time)
        thumb_is_left  = direction      (only used when fingers > 0)
        """
        four = sum(
            1 for tip in [8, 12, 16, 20]
            if lms.landmark[tip].y < lms.landmark[tip - 2].y
        )
        thumb_ext  = lms.landmark[4].x < lms.landmark[17].x
        thumb_left = lms.landmark[4].x < lms.landmark[5].x
        return four, thumb_ext, thumb_left

    def processing_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        res   = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        four      = 0
        thumb_ext = False
        thumb_left = False

        if res.multi_hand_landmarks:
            lms = res.multi_hand_landmarks[0]
            mp_draw.draw_landmarks(frame, lms, mp_hands.HAND_CONNECTIONS)
            four, thumb_ext, thumb_left = self._detect(lms)

        # Separate smoothing for fingers and thumb
        self.finger_history.pop(0)
        self.finger_history.append(four)
        smoothed = max(set(self.finger_history), key=self.finger_history.count)

        self.thumb_history.pop(0)
        self.thumb_history.append(thumb_ext)
        # Thumb needs majority (3/5) to change state — prevents flickering
        grip_closed = self.thumb_history.count(True) >= 3

        # ── GESTURE MAP ───────────────────────────────────────────────────────
        #
        # FINGER COUNT  → WHICH JOINT          THUMB DIRECTION  → WHICH WAY
        # ─────────────────────────────────────────────────────────────────────
        # 0 fingers     → FREEZE (hold pos)
        # 1 finger      → base              thumb_left=True → left, else right
        # 2 fingers     → shoulder          thumb_left=True → down, else up
        # 3 fingers     → elbow             thumb_left=True → in,   else out
        # 4 fingers     → HOME (all → 0)
        #
        # THUMB EXTENDED (independent, any time):
        #   True  → gripper CLOSED
        #   False → gripper OPEN
        #
        # BENEFIT: You can close the gripper mid-move by sticking your thumb
        # out while still showing 1-3 fingers. Fully independent axes.
        # ─────────────────────────────────────────────────────────────────────

        if smoothed == 0:
            pass   # Freeze — arm holds current position

        elif smoothed == 1:
            if thumb_left: self.target_positions[0] -= STEP_SIZE   # base left
            else:          self.target_positions[0] += STEP_SIZE   # base right

        elif smoothed == 2:
            if thumb_left: self.target_positions[1] -= STEP_SIZE   # shoulder down
            else:          self.target_positions[1] += STEP_SIZE   # shoulder up

        elif smoothed == 3:
            if thumb_left: self.target_positions[2] -= STEP_SIZE   # elbow in
            else:          self.target_positions[2] += STEP_SIZE   # elbow out

        elif smoothed == 4:
            # HOME — reset arm joints, gripper follows thumb as usual
            self.target_positions[0] = 0.0
            self.target_positions[1] = 0.0
            self.target_positions[2] = 0.0

        # Gripper: always driven by thumb, regardless of finger count
        self.target_positions[3] = GRIPPER_MAX_RAD if grip_closed else 0.0

        # ── KINEMATICS ────────────────────────────────────────────────────────
        self.target_positions = apply_dynamic_kinematics(self.target_positions)

        # ── VELOCITY-CAPPED SMOOTHING ─────────────────────────────────────────
        for i in range(4):
            diff = self.target_positions[i] - self.current_positions[i]
            step = np.clip(diff * SMOOTHING, -MAX_STEP, MAX_STEP)
            self.current_positions[i] += step

        # ── UDP PACKET ────────────────────────────────────────────────────────
        arm_angles    = [int(np.degrees(self.current_positions[i]) + 90) for i in range(3)]
        gripper_angle = gripper_to_servo_deg(self.current_positions[3])
        data_str      = ",".join(map(str, arm_angles + [gripper_angle]))

        self.get_logger().info(
            f"[{data_str}] | fingers={smoothed} thumb={'OUT' if grip_closed else 'IN'} "
            f"grip={'CLOSED' if grip_closed else 'OPEN'}"
        )

        # ── SYNC SIMULATION ───────────────────────────────────────────────────
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        pt = JointTrajectoryPoint()
        pt.positions       = list(self.current_positions)
        pt.time_from_start = Duration(sec=0, nanosec=50_000_000)
        msg.points.append(pt)
        self.publisher_.publish(msg)

        # ── SEND TO ESP ───────────────────────────────────────────────────────
        try:
            self.sock.sendto(data_str.encode(), (self.esp_ip, self.udp_port))
        except OSError as e:
            self.get_logger().warn(f"UDP send failed: {e}")

        # ── OVERLAY ───────────────────────────────────────────────────────────
        joint_labels = {0: "FREEZE", 1: "BASE", 2: "SHOULDER", 3: "ELBOW", 4: "HOME"}
        dir_label    = ("LEFT/DOWN/IN" if thumb_left else "RIGHT/UP/OUT") if smoothed in [1,2,3] else ""
        grip_label   = "CLOSED" if grip_closed else "OPEN"

        cv2.rectangle(frame, (0, 0), (frame.shape[1], 36), (20, 20, 40), -1)
        cv2.putText(frame,
            f"Joint: {joint_labels.get(smoothed,'?')}  Dir: {dir_label}  Grip: {grip_label}",
            (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
            (100, 255, 180) if smoothed > 0 else (150, 150, 150), 1)

        cv2.rectangle(frame, (0, frame.shape[0]-28), (frame.shape[1], frame.shape[0]), (20,20,40), -1)
        cv2.putText(frame, f"Pkt: {data_str}",
            (8, frame.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)

        # Gesture hint column
        hints = ["0: FREEZE", "1: base L/R", "2: shoulder U/D", "3: elbow I/O", "4: HOME"]
        for i, h in enumerate(hints):
            col = (100, 255, 180) if i == smoothed else (80, 80, 80)
            cv2.putText(frame, h, (frame.shape[1]-150, 60 + i*22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1)
        cv2.putText(frame, "thumb=grip", (frame.shape[1]-150, 60+5*22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                    (100, 255, 180) if grip_closed else (80, 80, 80), 1)

        cv2.imshow("Bot Gesture Controller", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.sock.close()
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = BotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()