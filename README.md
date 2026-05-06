# ROS 2 Line-Following Robot (Project B)

This repository contains the implementation of an autonomous differential-drive line-following robot using **ROS 2 Humble** and an **ESP32** microcontroller. This project is part of the Robotics Engineering course assessment.

## 🚀 Project Overview
The goal is to design and build a robot capable of:
- Sensing a dark line using a 5-IR sensor array.
- Processing sensor data at a minimum of 20Hz.
- Implementing a PID controller for smooth navigation.
- Estimating pose and trajectory via encoder-based odometry.
- Providing a web-based dashboard for remote monitoring.

## 📁 Repository Structure
- `src/`: ROS 2 packages (`line_follower_core`).
- `turtle_web_control/`: Prototype web interface.
- `implementation_plan.md`: Step-by-step development roadmap.
- `robotics_project_document.md`: Official project requirements and rubric.

## 🛠 Tech Stack
- **Middleware:** ROS 2 Humble Hawksbill
- **Microcontroller:** ESP32 (running Micro-ROS)
- **Simulation:** Gazebo (Project B digital twin)
- **Frontend:** HTML5/CSS3/JavaScript (using ROSlibJS)

## 📖 Getting Started
Refer to the [Implementation Plan](implementation_plan.md) for detailed instructions on building the simulation and hardware.

### Prerequisites
- ROS 2 Humble installed on Ubuntu 22.04.
- Micro-ROS Agent installed.
- ESP-IDF or Arduino IDE with Micro-ROS support.

## ⚖️ License
This project is developed for academic purposes as part of the Robotics Engineering 2025/2026 course.
