# ROS 2 Line-Following Robot (Project B)

This repository contains the implementation of an autonomous differential-drive line-following robot using **ROS 2 Humble** and an **ESP32** microcontroller.

## 🚀 Features
- **Dual Mode**: Independent launch files for [Hardware](src/line_follower_core/launch/line_follow.launch.py) and [Simulation](src/line_follower_core/launch/sim.launch.py).
- **Micro-ROS Integration**: Wireless bridge between ESP32 and Laptop via UDP.
- **Web Dashboard**: Premium real-time telemetry (LED indicators, velocity plots, PID tuning).
- **System Observability**: Built-in "System Graph" to monitor active nodes and topics in the browser.
- **Centralized Logic**: All PID control and odometry processing run on the host laptop.

---

## 🛠 Prerequisites & Installation

### 1. ROS 2 Dependencies
Ensure you have ROS 2 Humble and the necessary simulation/bridge packages installed:
```bash
sudo apt update
# Install Gazebo & ROS 2 Integration
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-robot-state-publisher
# Install ROSBridge and ROSAPI for the Web Dashboard
sudo apt install ros-humble-rosbridge-server ros-humble-rosapi
```

### 2. Micro-ROS Agent (for Hardware Mode)
The agent is required to bridge the ESP32 to the ROS 2 graph. Use Docker for the easiest setup:
```bash
sudo docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
```

---

## 🏗 Setup & Build

1. **Clone and Build the Workspace**:
   ```bash
   # In your ROS 2 workspace
   colcon build --packages-select line_follower_core
   source install/setup.bash
   ```

2. **Upload ESP32 Firmware**:
   - Navigate to `esp32_firmware/`.
   - Update `SSID`, `PASSWORD`, and `AGENT_IP` in `esp32_firmware.ino`.
   - Flash the code to your ESP32.

---

## 🚦 Running the System

### Mode A: Gazebo Simulation
Starts the digital twin with the line track and simulation nodes:
```bash
ros2 launch line_follower_core sim.launch.py
```

### Mode B: Physical Robot (Hardware)
1. **Start the Agent** (in a separate terminal):
   ```bash
   sudo docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
   ```
2. **Start the Hardware Stack**:
   ```bash
   ros2 launch line_follower_core line_follow.launch.py
   ```

---

## 📊 Web Dashboard
The launch files automatically start a local web server at `http://localhost:8000`.
- **System Graph**: Monitor active nodes and topics.
- **Hardware Debug**: View raw sensor data and motor PWM commands.
- **Mission Control**: Toggle manual override and tune PID parameters at runtime.

---

## 🔧 Troubleshooting

### "Address already in use" (Port 9090)
If the ROSBridge server fails to start because port 9090 is busy, kill the existing process:
```bash
sudo fuser -k 9090/tcp
```

### "Service /rosapi/nodes does not exist"
Ensure you have installed the `ros-humble-rosapi` package and that the `rosapi_node` is launched (included in the default launch files).

---

## ⚖️ License
Developed for the Robotics Engineering 2025/2026 course.
