# Implementation Plan: ROS 2 Line-Following Robot (Project B)

This implementation plan outlines a step-by-step approach to building the ROS 2 Humble Line-Following Robot. It covers both the **Gazebo simulation** and **physical hardware implementation** with an **ESP32 microcontroller**. The plan is highly modularized into distinct phases so that any AI tool can seamlessly follow, track progress, and implement it phase by phase.

*Note: This architecture and workflow are designed to be compatible with your future integration of Speckit.*

---

## Phase 1: Environment Setup & Architecture
**Goal:** Initialize the ROS 2 Humble workspace, core packages, and version control.
* **Task 1.1 - Workspace Creation:** Create a `ros2_ws` with the standard `src` directory.
* **Task 1.2 - Package Initialization:** Create a ROS 2 package `line_follower_core`. All nodes must be implemented using the **Object-Oriented (OOP)** structure (inheriting from `rclpy.Node`) as demonstrated in your `src/pub_sub` example, including proper use of timers and logger info.
* **Task 1.3 - Dependencies:** Add dependencies (`geometry_msgs`, `std_msgs`, `nav_msgs`, `sensor_msgs`, and `rclpy`/`rclcpp`).
* **Task 1.4 - Build & Validation:** Ensure the package builds successfully using `colcon build` and initialize a Git repository. *(Note: Maintain progressive Git commits throughout all phases to satisfy grading requirements).*

## Phase 2: Robot Modeling & Gazebo Simulation (URDF)
**Goal:** Create a simulated digital twin of the differential-drive robot for initial testing.
* **Task 2.1 - URDF Core:** Define the chassis, left drive wheel, right drive wheel, and caster wheel(s) using URDF/Xacro.
* **Task 2.2 - Differential Drive Plugin:** Attach the Gazebo differential drive plugin to simulate vehicle kinematics and publish simulated odometry (`/odom`).
* **Task 2.3 - Sensor Plugins:**
  * Add a simulated line sensor array (using multiple downward-facing ray sensors or a downward camera) publishing at a minimum rate of 20 Hz.
  * Add wheel encoder plugins.
* **Task 2.4 - World Generation:** Create a custom Gazebo world (`line_track.world`) featuring a closed-loop line track with at least two 90-degree turns and an 80cm straight segment.
* **Task 2.5 - Launch File:** Create `sim.launch.py` to spawn the robot in the custom Gazebo world.

## Phase 3: Core ROS 2 Nodes (Simulation & Logic)
**Goal:** Implement the primary software nodes required by the project specifications.
* **Task 3.1 - `line_sensor_node`:** Process simulated IR array data and publish a normalized lateral centroid error (`std_msgs/Float32`) to `/line_error` at a minimum rate of 20 Hz.
* **Task 3.2 - `encoder_odometry_node`:** Compute unicycle kinematic odometry from wheel encoders and publish to `/odom` (using `nav_msgs/Odometry`) at a minimum rate of 20 Hz.
* **Task 3.3 - `line_controller_node`:** Implement a Proportional (or PID) controller that subscribes to `/line_error` and publishes velocity commands (`geometry_msgs/Twist`) to `/cmd_vel`.
* **Task 3.4 - Simulation Tuning:** Tune the controller gains in the Gazebo simulation until the digital twin completes two consecutive laps autonomously.

## Phase 4: Hardware Integration & ESP32 Firmware
**Goal:** Interface the physical hardware components with the ESP32 microcontroller.
* **Task 4.1 - Micro-ROS Setup:** Set up `micro-ROS` for ESP32 (using the ESP-IDF or Arduino component). Configure the transport to use Wi-Fi (UDP) for wireless data transmission.
* **Task 4.2 - Sensor Firmware:** Write routines to read the 5-IR sensor array and wheel encoders (using high-resolution ESP32 timers/interrupts). Note: Ensure 3.3V logic compatibility for all sensor inputs.
* **Task 4.3 - Actuator Firmware:** Interface the ESP32 with the L298N motor driver using MCPWM or LEDC (PWM) peripherals. Define PWM duty cycle limits in software to prevent motor stall.
* **Task 4.4 - Communication Bridge:** Configure the ESP32 to publish `/line_error` and `/odom` wirelessly to the ROS 2 host at 20 Hz, and subscribe to `/cmd_vel` directly.
* **Task 4.5 - Safety & Indicators:** Implement an observable hardware emergency-stop (E-stop) button. Add an indicator (built-in LED or Buzzer) to signal lap completion.

## Phase 5: Physical Robot ROS 2 Integration
**Goal:** Complete the integration between the physical Arduino and the ROS 2 host computer (e.g., Raspberry Pi or Laptop).
* **Task 5.1 - Node Adaptation:** Adapt the host-side nodes to connect to the ESP32 via the Micro-ROS agent. Ensure the laptop/Pi is on the same Wi-Fi network as the ESP32.
* **Task 5.2 - Calibration Routine:** Implement the required line calibration script to distinguish the dark line from the light ground based on ambient lighting.
* **Task 5.3 - Hardware Launch File:** Create `line_follow.launch.py` to launch all physical nodes, micro-ROS agent, and set PID controller gains via ROS parameters.

## Phase 6: Physical Testing & PID Tuning
**Goal:** Achieve stable, autonomous line-following in the real world.
* **Task 6.1 - Component Validation:** Test individual components (verify motors turn in the correct direction, IR sensors respond to the track, E-stop halts motors, and lap indicator triggers).
* **Task 6.2 - PID Tuning:** Methodically tune the P, I, and D parameters on the physical track.
* **Task 6.3 - Tuning Log:** Record a table of tested gains and the resulting lap times, maximum lateral deviation, and cross-track RMS error as requested in the rubric.
* **Task 6.4 - ROS Bag Recording:** Record `/line_error`, `/cmd_vel`, and `/odom` for at least two successful physical laps using `ros2 bag record`.

## Phase 7: Data Analysis & Final Reporting
**Goal:** Generate the required kinematics plots and compile the final deliverables.
* **Task 7.1 - Odometry Plotting:** Extract odometry data from the ROS bag and plot the estimated trajectory (x, y) against the actual physical track layout.
* **Task 7.2 - Kinematics Documentation:** Document the differential-drive unicycle kinematic model, forward kinematics (odometry integration), and PID controller equations. Ensure wheel encoder resolution is explicitly stated in pulses per revolution (PPR).
* **Task 7.3 - Repository Finalization:** Add the required `README.md` with comprehensive build and run instructions. Ensure Git history reflects progressive development across the project lifespan.
* **Task 7.4 - Demonstration Video:** Finalize the 3–5 minute video showcasing the autonomous physical laps, narrated code logic, E-stop functionality, and lap completion indicator.
* **Task 7.5 - Formal Technical Report:** Draft the final 15–35 page PDF report encompassing: Title Page, Abstract, System Description, Sensing, Actuation, ROS Architecture, Kinematics, Experimental Results, Discussion, Conclusion, References, and Appendix (as strictly defined in Section 7.1). Ensure references are in IEEE format (minimum 5), and the Appendix includes a disclosure statement for any AI/LLM tools used during development.
## Phase 8: Web Dashboard & Remote Monitoring (GUI)
**Goal:** Create a web-based interface for real-time monitoring and manual control of the robot.
* **Task 8.1 - Web Interface Setup:** Create a `gui` directory within the package. Implement a modern HTML5/CSS3 dashboard (similar to `turtle_web_control`) with a responsive layout.
* **Task 8.2 - ROSBridge Integration:** Configure `roslibjs` to connect to the `rosbridge_websocket` on the host laptop.
* **Task 8.3 - Telemetry Visualization:** Add real-time gauges or charts to visualize `/line_error` and robot speed from `/odom`.
* **Task 8.4 - Remote Command Panel:** Implement buttons for "Start Autonomous Lap", "Emergency Stop", and "Manual Drive Mode" (publishing to `/cmd_vel`).
* **Task 8.5 - Calibration UI:** Add a button to trigger the `line_sensor` calibration routine remotely from a phone or tablet.
