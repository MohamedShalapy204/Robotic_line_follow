# Implementation Plan: ROS 2 Line-Following Robot (Project B)

This implementation plan outlines a step-by-step approach to building the ROS 2 Humble Line-Following Robot. It covers both the **Gazebo simulation** and **physical hardware implementation** with an **ESP32 microcontroller**. The plan is highly modularized into distinct phases so that any AI tool can seamlessly follow, track progress, and implement it phase by phase.

*Note: This architecture and workflow are designed to be compatible with your future integration of Speckit.*

---

## Phase 1: Environment Setup & Architecture
**Goal:** Initialize the ROS 2 Humble workspace, core packages, and version control.
* **Task 1.1 - Workspace Creation:** Create a `ros2_ws` with the standard `src` directory.
* **Task 1.2 - Package Initialization:** Create a ROS 2 package `line_follower_core`. All nodes must be implemented using the **Object-Oriented (OOP)** structure (inheriting from `rclpy.Node`) as demonstrated in your `src/pub_sub` example, including proper use of timers and logger info.
* **Task 1.3 - Dependencies:** Add dependencies (`geometry_msgs`, `std_msgs`, `nav_msgs`, `sensor_msgs`, and `rclpy`/`rclcpp`).
* **Task 1.4 - Build & Validation:** Ensure the package builds successfully using `colcon build` and initialize a Git repository.

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

## Phase 4: Hardware Integration & WiFi Telemetry
**Goal:** Interface the physical hardware components with the host laptop via Wi-Fi.
* **Task 4.1 - Micro-ROS WiFi Setup:** Set up `micro-ROS` on ESP32 configured for Wi-Fi (UDP) transport.
* **Task 4.2 - Sensor Telemetry Firmware:** Implement raw IR sensor data (5 topics) and encoder data streaming from ESP32 to host at 20Hz+.
* **Task 4.3 - Actuator Interface Firmware:** Configure ESP32 to subscribe to motor speed/PWM topics and drive DC motors via L298N.
* **Task 4.4 - Safety Failsafe:** Implement a software timeout on the ESP32 that stops motors if no `/cmd_vel` message is received for > 500ms.
* **Task 4.5 - Unified Host Launch:** Create `line_follow.launch.py` on the laptop to launch the micro-ROS agent, `line_sensor_node`, `line_controller_node`, `encoder_odometry_node`, and `motor_driver_node`. Update `motor_driver_node` to publish motor commands to the ESP32.

## Phase 5: GUI Dashboard & Real-time Monitoring
**Goal:** Enhance the web interface to visualize all hardware telemetry and control the robot.
* **Task 5.1 - GUI Sensor Mapping:** Update `turtle_web_control` to display 5 IR sensor states and `/line_error`.
* **Task 5.2 - Odom Visualization:** Integrate real-time linear/angular velocity displays from `/odom` into the dashboard.
* **Task 5.3 - Calibration UI:** Add remote calibration triggers to the GUI.

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
