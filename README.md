# 🤖 ROS 2 Hardware-Only Line-Following Robot (Project B)

An advanced, high-performance, real-time autonomous line-following robot system powered by **ROS 2 Humble** on the host laptop and a wireless **micro-ROS** bridge running on an **ESP32** microcontroller.

---

## 📐 Mathematical & Kinematics Model

The system utilizes a **Differential-Drive Unicycle Kinematics Model** to translate host-side twist commands directly into physical H-bridge PWM actuation.

### Vehicle Specifications

* **Wheel Separation ($L$):** $0.135 \text{ meters}$ (measured baseline)
* **Wheel Radius ($r$):** $0.0325 \text{ meters}$
* **Encoder Resolution:** $20 \text{ Pulses Per Revolution (PPR)}$

### Inverse Kinematics Mapping

The target twist ($v, \omega$) from the PID controller is mapped into target wheel speeds ($\omega_L, \omega_R$ in $\text{rad/s}$):

$$
\omega_L = \frac{2v - \omega L}{2r}
$$

$$
\omega_R = \frac{2v + \omega L}{2r}
$$

The wheel speeds are scaled directly to 8-bit H-Bridge PWM commands via a tuned gain ($G_{\text{pwm}} = 20.0$) and constrained to $[-255, 255]$.

### Forward Kinematics (Odometry)

Encoder tick accumulation is integrated over time using a **2nd Order Runge-Kutta (Midpoint) Approximation** to estimate global coordinate pose $(x, y, \theta)$ with minimal drift:

$$
\Delta d_c = \frac{\Delta d_L + \Delta d_R}{2}
$$

$$
\Delta \theta = \frac{\Delta d_R - \Delta d_L}{L}
$$

$$
x_{t+1} = x_t + \Delta d_c \cos\left(\theta_t + \frac{\Delta \theta}{2}\right)
$$

$$
y_{t+1} = y_t + \Delta d_c \sin\left(\theta_t + \frac{\Delta \theta}{2}\right)
$$

---

---

## 🚀 Installation & Build

### 1. Install Host Dependencies

Ensure ROS 2 Humble is installed:

```bash
sudo apt update
```

### 2. Compile the Workspace

Navigate to your ROS 2 workspace, clone the package, and compile:

```bash
colcon build --packages-select line_follower_core
source install/setup.bash
```

### 3. Flash ESP32 Firmware

1. Open `esp32_firmware/esp32_firmware.ino` in Arduino IDE.
2. Update your network parameters (`ssid`, `password`, and `agent_ip`).
3. Build and upload the code to your ESP32 board.

---

## 🚦 Running the System

### Step 1: Start the micro-ROS UDP Agent

Run the standard micro-ROS agent on your laptop (port `8888` via UDP) to accept connections from the ESP32:

```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
```

### Step 2: Launch the ROS 2 Core Stack

In a new terminal with the workspace sourced, run the universal launch description:

```bash
ros2 launch line_follower_core line_follow.launch.py
```

---

## 📊 Troubleshooting

* **Packet Drops:** Ensure the laptop and ESP32 are connected to a high-speed, stable local Wi-Fi router.

---

*Developed for the Robotics Engineering Course.*
