# Kinematics Documentation

## Differential-Drive Unicycle Kinematic Model

The robot is modeled as a differential-drive unicycle. The relationship between the individual wheel velocities and the robot's center velocities (linear velocity $v$, angular velocity $\omega$) is:

- $v = \frac{v_R + v_L}{2}$
- $\omega = \frac{v_R - v_L}{L}$

Where:

- $v_R, v_L$ are the linear velocities of the right and left wheels.
- $L$ is the wheel separation (base-width), measured at $0.135$ meters.
- $r$ is the wheel radius, measured at $0.0325$ meters.

---

## Forward Kinematics (Odometry Integration)

The robot's pose $(x, y, \theta)$ in the global frame is estimated by integrating wheel encoder data over time.

**Encoder Specifications:**

- Resolution (PPR): **20 Pulses Per Revolution**

### 1. Angular displacement of the wheels

Given the encoder tick counts ($\Delta \text{ticks}$), the angular displacement in radians ($\Delta \phi$) is calculated as:

$$
\Delta \phi = \left( \frac{\Delta \text{ticks}}{\text{PPR}} \right) \times 2\pi
$$

### 2. Linear distance traveled by each wheel

$$
\Delta d_L = \Delta \phi_L \times r
$$

$$
\Delta d_R = \Delta \phi_R \times r
$$

### 3. Center distance and heading change

The distance traveled by the center of the robot ($\Delta d_c$) and the change in heading ($\Delta \theta$) are:

$$
\Delta d_c = \frac{\Delta d_L + \Delta d_R}{2}
$$

$$
\Delta \theta = \frac{\Delta d_R - \Delta d_L}{L}
$$

### 4. Pose Update (Runge-Kutta 2nd Order Approximation)

The robot's position is integrated using the midpoint of the heading change to reduce integration drift:

$$
x_{t+1} = x_t + \Delta d_c \cos\left(\theta_t + \frac{\Delta \theta}{2}\right)
$$

$$
y_{t+1} = y_t + \Delta d_c \sin\left(\theta_t + \frac{\Delta \theta}{2}\right)
$$

$$
\theta_{t+1} = \theta_t + \Delta \theta
$$

*(These equations are implemented in `encoder_odometry_node.py`)*

---

## PID Controller Equations

The lateral tracking error $e(t)$ is computed using a weighted centroid of the 5 IR sensors (`l2`, `l1`, `mid`, `r1`, `r2` with weights `2.0`, `1.0`, `0.0`, `-1.0`, `-2.0` respectively).

The controller applies a Proportional-Integral-Derivative (PID) algorithm to generate the required angular velocity $\omega(t)$:

$$
\omega(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}
$$

The linear velocity $v(t)$ is maintained at a constant `base_speed`, with optional kickstart pulses upon initial activation to overcome static friction. The target twist ($v, \omega$) is then inversely mapped back to left and right wheel PWM signals using the inverse of the unicycle model inside `motor_driver_node.py`.
