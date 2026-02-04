# Autonomous Self-Balancing Bicycle

This project explores the design and implementation of an autonomous, self-balancing bicycle using a reaction wheel for stabilization. The system combines embedded control, power electronics, and robotic autonomy to achieve balance, perception, and navigation on a dynamically unstable platform.

A reaction wheel driven by a high-speed motor provides roll stabilization, while steering, drive, and braking actuators enable controlled motion. Low-level balancing is handled by an embedded controller, with higher-level perception and autonomy running on an onboard computer using ROS2.

The autonomy stack integrates stereo vision, LiDAR, and inertial sensing to perform visual SLAM, localization, and obstacle-aware navigation. The project emphasizes tight integration between mechanical design, real-time control, and perception-driven decision making.

This repository contains the hardware designs, embedded firmware, autonomy software, and documentation developed in the course of building and testing the system.