# ESP32 Micro-ROS Firmware Instructions

This folder contains the firmware for the ESP32 that acts as a wireless bridge between the physical sensors/actuators and the ROS 2 host laptop.

## Prerequisites

1. **Arduino IDE** or **PlatformIO**.
2. **Micro-ROS Arduino Library**: Use the **Humble** branch of the library.
   - [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino/tree/humble)
3. **Board Support**: ESP32 Dev Module (ESP32-WROOM-32 recommended).

## ⚡ Hardware Warnings (Critical)

- **Common Ground**: Ensure the ESP32 GND and L298N GND are connected together.
- **External Power**: The L298N must be powered by a battery (e.g., 7.4V or 12V). Do NOT power motors from the ESP32 3.3V/5V pins.
- **USB Connection**: Keep the ESP32 plugged into the laptop during testing to use the Serial Monitor (115200 baud) for debug logs.

## Configuration

Before uploading, update the following variables in `esp32_firmware.ino`:

- `SSID`: Your Wi-Fi network name.
- `PASSWORD`: Your Wi-Fi password.
- `AGENT_IP`: The IP address of your host laptop running the `micro-ros-agent`.

## Pin Mapping

| Component | Pin (GPIO) | Mode |
| :--- | :--- | :--- |
| **IR Sensor L2** | 13 | Input |
| **IR Sensor L1** | 12 | Input |
| **IR Sensor MID** | 14 | Input |
| **IR Sensor R1** | 27 | Input |
| **IR Sensor R2** | 26 | Input |
| **Motor Left ENA** | 32 | PWM (Channel 0) |
| **Motor Left IN1** | 33 | Output |
| **Motor Left IN2** | 25 | Output |
| **Motor Right ENB** | 19 | PWM (Channel 1) |
| **Motor Right IN3** | 18 | Output |
| **Motor Right IN4** | 5 | Output |
| **Encoder Left** | 34 | Input (Interrupt) |
| **Encoder Right** | 35 | Input (Interrupt) |

## Testing Instructions

1. **Upload**: Flash the firmware to the ESP32.
2. **Start Agent**: On the laptop, run the Micro-ROS agent:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```
3. **Verify Topics**:
   - Check sensor data: `ros2 topic echo /raw/sensor_mid`
   - Test motors: `ros2 topic pub /motor/left/pwm std_msgs/msg/Int32 "{data: 150}"`
4. **Safety Check**: Unplug the laptop or stop the agent. The motors should stop automatically after 500ms (Heartbeat Failsafe).

## 🔍 Debugging
If the ESP32 is not connecting:
1. Open the Serial Monitor at **115200 baud**.
2. If it stays at "Connecting...", check if the **Laptop Firewall** is blocking UDP port 8888.
3. Ensure the `AGENT_IP` matches your laptop's Wi-Fi IP (check via `hostname -I`).
