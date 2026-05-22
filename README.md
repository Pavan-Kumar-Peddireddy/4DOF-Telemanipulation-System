# 🤖 Vision-Guided Telemanipulation System

A modular ROS 2 workspace implementing a **vision-guided telemanipulation framework** for a 4-DOF robotic manipulator. The system integrates:

- real-time computer vision,
- inverse kinematics,
- synchronized digital twin simulation,
- and embedded wireless actuation

to create a complete low-latency human-in-the-loop robotic control pipeline.

The workspace establishes a synchronized **Digital Twin architecture** where human hand gestures are interpreted in real time and mirrored simultaneously across:

- a physical robotic manipulator,
- Gazebo simulation,
- and RViz visualization nodes.

---

# 📂 Workspace Structure

```text
├── gesture_control_pkg/       # Vision & tracking nodes workspace
├── motion_programming_pkg/    # Kinematics & execution nodes workspace
├── media/                     # Centralized repository assets
│   ├── gesture_nodes/         # Tracking diagnostics & UI captures
│   │   ├── ui_screenshot.png
│   │   └── pipeline_demo.mp4
│   └── simulation/            # Gazebo & ROS graph recordings
│       ├── gazebo_execution.mp4
│       └── rqt_graph.png
└── README.md
```
# 🎥 Live Demonstrations

## ✋ Gesture Teleoperation

https://github.com/user-attachments/assets/gesture-demo-id

---

## 📦 Pick & Place Task

https://github.com/user-attachments/assets/pick-place-id

---

## 🧪 Gazebo Digital Twin

https://github.com/user-attachments/assets/gazebo-demo-id
---

# 👥 Contributors

| Role | Name |
|---|---|
| Institution | Parul Institute of Technology, Parul University |
| Department | Robotics and Automation Engineering |
| Project Members | Peddireddy Pavan Kumar |
|  | Velimineti Guru Charan Reddy |
|  | Singireddy Karthik |
| Project Advisor | Dr. SSPM Sharma B |

---

# 🏗️ System Architecture

The workspace follows a modular ROS 2 architecture divided into four major subsystems.

---

## 1️⃣ Perception Layer — `gesture_control_pkg`

Captures and processes human hand gestures in real time using computer vision.

### Features

- Webcam-based gesture acquisition
- MediaPipe hand landmark tracking
- OpenCV preprocessing pipeline
- ROS 2 topic-based data publishing
- Real-time coordinate extraction

### Technologies Used

- ROS 2 Humble
- Python
- OpenCV
- MediaPipe

---

## 2️⃣ Kinematics & Control Layer

Responsible for converting spatial gesture coordinates into robotic joint trajectories.

### Features

- Forward Kinematics (FK)
- Inverse Kinematics (IK)
- DH Parameter Modeling
- Real-time joint angle computation
- Smooth trajectory interpolation
- Servo command generation

---

## 3️⃣ Visualization & Digital Twin Layer

Synchronizes all robotic states across visualization and simulation environments.

### Components

- RViz visualization
- Gazebo physics simulation
- Joint state broadcasters
- TF transformation tree

### Benefits

- Real-time trajectory validation
- Safer testing environment
- Simulation-assisted debugging
- Digital twin synchronization

---

## 4️⃣ Embedded Actuation Layer

Handles real-time robotic hardware execution using an ESP32 microcontroller.

### Features

- UDP socket communication
- Wi-Fi based wireless control
- PWM servo actuation
- Low-latency command streaming
- Motion smoothing firmware

### Hardware Communication Stack

```text
ROS 2 Node → UDP Socket → ESP32 → PCA9685 → SG90 Servos
```

---

# 📐 Hardware Specifications

| Component | Specification |
|---|---|
| Manipulator Type | 4-DOF Robotic Arm |
| Joint Configuration | Base, Shoulder, Elbow, Gripper |
| Base Height (`BASE_H`) | 6.5 cm |
| Link 1 (`L₁`) | 9.3 cm |
| Link 2 (`L₂`) | 11.0 cm |
| Servo Motors | 4× SG90 Coreless Servo Motors |
| PWM Driver | PCA9685 |
| Main Controller | ESP32 Dual-Core MCU |
| Power Regulation | LM2596 Buck Converter |
| Chassis Material | PLA / PETG |
| CAD Environment | Fusion 360 |

---

# ⚙️ Core Software Features

---

## 🛡️ Dynamic Workspace Boundary Protection

Implements runtime workspace validation to prevent:

- manipulator overextension,
- self-collision,
- ground collision,
- and unstable configurations.

### Operational Constraints

| Parameter | Value |
|---|---|
| Maximum Reach | 18.0 cm |
| Minimum Reach | 6.5 cm |

---

## 🎮 Gesture-Based Teleoperation

Enables intuitive robotic manipulation through natural human hand gestures tracked via computer vision.

### Capabilities

- Real-time hand tracking
- Spatial gesture interpretation
- Continuous motion control
- Low-latency command execution

---

## 🔁 Trajectory Recording & Replay

Supports robotic motion programming through:

- waypoint recording,
- trajectory serialization,
- and smooth replay execution.

### Applications

- Learning from Demonstration (LfD)
- Robotic task automation
- Repeatable motion execution

---

## ⚡ Low-Latency Motion Engine

Custom firmware-level smoothing algorithms reduce:

- abrupt servo acceleration,
- oscillation,
- and mechanical stress during operation.

---

# 🔄 Data Flow Pipeline

```text
Webcam Feed
      ↓
MediaPipe Landmark Detection
      ↓
Gesture Coordinate Extraction
      ↓
Inverse Kinematics Solver
      ↓
ROS 2 Topic Publishing
      ↓
 ┌───────────────┬─────────────────┬────────────────┐
 ↓               ↓                 ↓
RViz         Gazebo Sim         ESP32 Hardware
Visualization  Digital Twin      Physical Arm
```

---

# 🧠 Kinematic Model

The manipulator uses a custom Denavit-Hartenberg (DH) parameter model for real-time forward and inverse kinematic calculations.

## Supported Computations

- End-effector position estimation
- Joint angle derivation
- Workspace boundary validation
- Reachability analysis

---

# 💻 Software Stack

| Layer | Technologies |
|---|---|
| Robotics Middleware | ROS 2 Humble |
| Vision Processing | OpenCV, MediaPipe |
| Simulation | Gazebo |
| Visualization | RViz2 |
| Embedded Control | ESP32 |
| Programming Language | Python |
| CAD Design | Fusion 360 |

---

# 🚀 Workspace Setup

---

## 1️⃣ Prerequisites

### Ubuntu & ROS 2

Recommended environment:

- Ubuntu 22.04 LTS
- ROS 2 Humble

Install ROS 2 Humble:

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

---

## 2️⃣ Clone the Repository

```bash
cd ~/your_ws/src
git clone <your_repository_url>
```

---

## 3️⃣ Install Dependencies

```bash
pip install mediapipe opencv-python numpy
```

Install ROS dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 4️⃣ Build the Workspace

```bash
cd ~/your_ws
colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

---

# ▶️ Running the System

---

## 1️⃣ Launch Gesture Tracking

```bash
ros2 launch gesture_control_pkg gesture_tracking.launch.py
```

---

## 2️⃣ Launch Gazebo Simulation

```bash
ros2 launch motion_programming_pkg simulation.launch.py
```

---

## 3️⃣ Launch RViz Visualization

```bash
ros2 launch motion_programming_pkg rviz.launch.py
```

---

## 4️⃣ Start Hardware Communication

Ensure:

- ESP32 and host machine are connected to the same Wi-Fi network
- UDP ports are configured correctly
- External servo power is stable

Then run:

```bash
ros2 run motion_programming_pkg udp_bridge_node
```

---

# 🖼️ Demonstration

---

## Gesture Tracking Interface

![Gesture Tracking](media/gesture_nodes/ui_screenshot.png)

---

## Gazebo Digital Twin

![Gazebo Simulation](media/simulation/rqt_graph.png)

---

# 📊 ROS 2 Node Graph

Example node communication structure:

```text
gesture_tracking_node
        ↓
inverse_kinematics_node
        ↓
trajectory_controller_node
   ┌─────────────┬───────────────┐
   ↓             ↓               ↓
rviz_node    gazebo_node    udp_bridge_node
```

---

# 🔬 Engineering Highlights

- Modular ROS 2 distributed architecture
- Real-time digital twin synchronization
- Wireless embedded robotic actuation
- Vision-guided teleoperation
- Physics-backed simulation environment
- Motion trajectory recording & replay
- Real-time inverse kinematics computation

---

# 📈 Future Improvements

- 6-DOF manipulator extension
- MoveIt 2 integration
- Reinforcement Learning trajectory optimization
- Depth camera integration
- SLAM-assisted teleoperation
- Haptic feedback system
- Multi-camera gesture fusion
- AI-assisted motion prediction

---

# 🛠️ Troubleshooting

---

## ROS 2 Package Not Found

Run:

```bash
source install/setup.bash
```

---

## Gazebo Not Launching

Install Gazebo dependencies:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

---

## MediaPipe Camera Error

Check webcam permissions:

```bash
ls /dev/video*
```

---

## Servo Jitter Issues

- Use external regulated power supply
- Ensure common ground connection
- Reduce command update frequency if necessary

---

# 📜 License

```text
MIT License

Copyright (c) 2026

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files...
```

---

# 🙏 Acknowledgements

Special thanks to:

- ROS 2 Community
- OpenCV Contributors
- MediaPipe Team
- Gazebo Simulation Ecosystem
- RViz Developers
- ESP32 Open Hardware Community
- Parul University Robotics Department

---

# ⭐ Support

If you found this project useful:

- Star the repository
- Fork the project
- Contribute improvements
- Share feedback

---

# 📬 Contact

For collaborations or technical discussions:

- Robotics & Automation Engineering Department
- Parul Institute of Technology
- Parul University

---
