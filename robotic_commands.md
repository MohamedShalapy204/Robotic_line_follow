# 🤖 Robotic Line Follower - ROS 2 CLI Commands Cheat Sheet

Welcome to the command center cheat sheet! Since we have optimized the robot to run **100% locally and automatically on boot**, you can control, monitor, and tune your robot directly via the ROS 2 Command Line Interface (CLI).

---

## ⚡ 1. How to Run the Robot

Follow these simple steps in order to start your autonomous robotic system:

### Step 1: Upload the Firmware
* Open **[esp32_firmware.ino](file:///home/omnia/Robotic_line_follow/esp32_firmware/esp32_firmware.ino)** in **Arduino IDE**.
* Connect your ESP32 to your PC via USB cable.
* Select the correct port (e.g. `/dev/ttyUSB0`) and upload the sketch.

### Step 2: Start the micro-ROS UDP Agent
Open a **new terminal** and run the UDP Wi-Fi agent. Once the robot connects to your local Wi-Fi, it will automatically register and start moving:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
> [!NOTE]
> The LED on the ESP32 will transition from **flashing** to a **solid light** once it successfully binds to the agent.

### Step 3: Launch ROSBridge Server (Optional)
Open a **second terminal** and launch the bridge description:
```bash
cd /home/omnia/Robotic_line_follow
source install/setup.bash
ros2 launch line_follower_core line_follow.launch.py
```

---

## ⚙️ 2. Dynamic Real-Time Parameter Tuning

You can adjust PID coefficients, base speeds, wheel trim factors, and sensor thresholds on-the-fly without stopping the robot or rebuilding! 

Publish a JSON string payload to the `/tuning_params` topic:

### 🎯 Tune PID Coefficients (Gain settings)
Increase `kp` for faster turn correction, or increase `kd` to damp high-frequency oscillation:
```bash
ros2 topic pub --once /tuning_params std_msgs/msg/String "{data: '{\"kp\":1.4,\"ki\":0.0,\"kd\":0.15}'}"
```

### 🏎️ Adjust Base Linear Speed
To make the robot run faster or slower on straight segments:
```bash
ros2 topic pub --once /tuning_params std_msgs/msg/String "{data: '{\"base_speed\":0.35}'}"
```

### 💡 Adjust Sensor Line Threshold
If lighting conditions change, modify the white-line detection threshold (less than this value counts as the white line):
```bash
ros2 topic pub --once /tuning_params std_msgs/msg/String "{data: '{\"sensor_threshold\":2200}'}"
```

### ⚖️ Adjust Motor Speeds Balance (Wheel Trim Factor)
Calibrate Left and Right motor PWM scaling factors dynamically:
```bash
ros2 topic pub --once /tuning_params std_msgs/msg/String "{data: '{\"left_trim\":1.15,\"right_trim\":1.0}'}"
```

### 🌀 Full Parameter Tuning (All-in-one)
```bash
ros2 topic pub --once /tuning_params std_msgs/msg/String "{data: '{\"kp\":1.5,\"ki\":0.0,\"kd\":0.12,\"base_speed\":0.4,\"sensor_threshold\":1900,\"left_trim\":1.143}'}"
```

---

## 📊 3. Live Telemetry Monitoring

Monitor what the robot is thinking and seeing in real-time:

### 📈 Watch Live Line Error (Weighted deviation)
Outputs values from `-2.0` (far left) to `2.0` (far right). `0.0` represents perfectly centered:
```bash
ros2 topic echo /line_error
```

### 🧭 Watch Real-Time Position & Speed (Odometry)
Echoes continuous unicycle-model pose estimations ($x$, $y$, heading $\theta$) and velocities ($v$, $\omega$):
```bash
ros2 topic echo /odom
```
