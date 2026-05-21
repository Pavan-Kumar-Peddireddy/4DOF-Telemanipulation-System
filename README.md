# 🤖 Vision-Guided Telemanipulation System

[cite_start]A complete, multi-package ROS 2 engineering workspace integrating computer vision, real-time physics simulation, and embedded hardware control for an agile 4-Degree-of-Freedom (4-DOF) robotic manipulator[cite: 484, 485]. 

[cite_start]This project establishes a synchronized **Digital Twin** framework[cite: 488]. [cite_start]Commands extracted from a human operator's hand gestures are processed in real-time and broadcast simultaneously to a physical robotic arm, a Gazebo physical simulation, and an RViz visualization node[cite: 487, 488, 606].

---

## 👥 Project Contributors & Identity
* [cite_start]**Institution:** Parul Institute of Technology, Parul University [cite: 435]
* [cite_start]**Department:** Department of Robotics and Automation Engineering [cite: 432]
* [cite_start]**Project Members:** Peddireddy Pavan Kumar, Velimineti Guru Charan Reddy, Singireddy Karthik [cite: 421, 422, 423]
* [cite_start]**Project Advisor:** Dr. SSPM Sharma B [cite: 430]

---

## 🛠️ System Architecture

[cite_start]The workspace follows a highly modular ROS 2 architecture split into critical runtime nodes[cite: 485, 532]:

1.  [cite_start]**Perception Layer (`gesture_control_pkg`)**: Uses a standard webcam feed to capture human gestures, tracking hand landmarks through a computer vision pipeline (OpenCV/MediaPipe)[cite: 485, 529, 601].
2.  [cite_start]**Kinematics & Control Layer**: A Python-driven core computing real-time Forward and Inverse Kinematics (DH parameters) to translate spatial coordinate parameters into exact joint angular vectors[cite: 541, 542].
3.  [cite_start]**Visualization & Digital Twin Sync**: Broadcasters that mirror runtime trajectories across `RViz` and `Gazebo Sim` simultaneously[cite: 487, 488, 606].
4.  [cite_start]**Embedded Actuation Layer**: Relies on a User Datagram Protocol (UDP) socket connection over Wi-Fi to pipe commands directly to an ESP32 microcontroller at minimum latency[cite: 486, 533].

---

## 📐 Hardware Specifications & Mechatronics

* [cite_start]**Manipulator Kinematics:** 4-DOF structure consisting of Base Rotation, Shoulder Pitch, Elbow Pitch, and a functional End-Effector Gripper[cite: 484, 214].
* [cite_start]**Link Dimensions:** Base Height ($BASE_H$) = 6.5 cm, Link 1 ($L_1$) = 9.3 cm, Link 2 ($L_2$) = 11.0 cm[cite: 216].
* [cite_start]**Actuation System:** 4x SG90 Coreless Servo Motors managed via a PCA9685 PWM driver matrix[cite: 209, 221].
* [cite_start]**Primary Controller:** Dual-core ESP32 processing unit receiving low-latency wireless payloads[cite: 218].
* [cite_start]**Power Subsystem:** High-capacity Li-ion cell bank stepped down smoothly via an LM2596 DC-DC buck converter[cite: 224].
* [cite_start]**Chassis Composition:** Custom lightweight structural chassis components modeled in Fusion 360 and built using 3D-printed PLA/PETG materials[cite: 215, 216].

---

## 💾 Core Software Features

* [cite_start]**Dynamic Boundary Guarding:** Safe tracking algorithms enforcing workspace operational boundaries (Max reach: 18.0 cm, Min reach: 6.5 cm) paired with ground-level constraints to eliminate structural collisions or tipping[cite: 257, 409].
* [cite_start]**Trajectory Record & Replay:** A motion programming node enabling operators to save sequential trajectory positions to a database and re-execute them smoothly, forming the basis for Learning-from-Demonstration (LfD) routines[cite: 490].
* [cite_start]**Low-Latency Control Engine:** Microcontroller firmware built with a custom smoothing engine to prevent abrupt motor stress during sharp teleoperation movements[cite: 398].

---

## 🚀 Workspace Setup & Quickstart

### 1. Compilation
Make sure your ROS 2 environment is sourced correctly on Ubuntu, clone this repository directly into your source directory, and compile:
```bash
cd ~/your_ws
colcon build --symlink-install
source install/setup.bash
