# 🚦 Complete Guide: Run Robot, Record Bag & Plot Trajectory

This guide provides a step-by-step procedure to run the robot, record its odometry data into a ROS 2 bag file, and generate an interactive plot of the trajectory using `plot_odometry.py`.

---

## 🏗️ Overview of Terminals
To run this complete pipeline, you will need **3 terminal windows** open at the same time:
1. **Terminal 1:** Runs the micro-ROS agent (connects the ESP32 to ROS 2).
2. **Terminal 2:** Launches your odometry node.
3. **Terminal 3:** Records the ROS 2 bag or runs the playback & plotting scripts.

---

## 🚦 Phase 1: Running the Robot & Recording Data

### Step 1: Start the micro-ROS Agent (Terminal 1)
Open your first terminal and run the micro-ROS agent:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
> [!NOTE]
> Turn on your robot. The LED on the ESP32 will transition from **flashing** to a **solid light** once it successfully binds to this agent.

---

### Step 2: Launch the Odometry Node (Terminal 2)
Open your second terminal, source your workspace, and launch the customized launch file (which now only runs the `encoder_odometry_node`):
```bash
cd ~/Robotic_line_follow
source install/setup.bash
ros2 launch line_follower_core line_follow.launch.py
```

---

### Step 3: Record the ROS 2 Bag (Terminal 3)
Open your third terminal, make the recording script executable, and run it:
```bash
cd ~/Robotic_line_follow
chmod +x record_bag.sh
./record_bag.sh
```
* Let the robot complete at least **2 full laps** on the track.
* Once completed, press **`Ctrl + C`** in Terminal 3 to stop recording.
* Your recording is saved as a new directory inside `./rosbags/` (e.g., `rosbags/lap_recording_20260519_070512`).

---

## 📊 Phase 2: Generating the Trajectory Plot

Now that you have recorded the bag, you can play it back to generate the plot.

### Step 4: Start the Plotter (Terminal 3)
In your third terminal, run the plotter script:
```bash
python3 plot_odometry.py
```
It will display: `Listening to /odom and /line_error...` and wait for you to play the bag.

---

### Step 5: Play Back the Bag File (Terminal 2)
In your second terminal (or any terminal where ROS 2 is sourced), play the bag file:

> [!IMPORTANT]
> **Avoid copy-pasting `<TIMESTAMP>` literally!** Doing so will cause a bash syntax error because of the `<` and `>` characters.
> 
> Instead, use **Tab Completion**:
> 1. Type: `ros2 bag play rosbags/lap_recording_`
> 2. Press the **`Tab`** key on your keyboard. 
> 3. Bash will automatically fill in the correct timestamp folder for you!
> 4. Press **`Enter`** to start playback.

For example, your command will look like this:
```bash
ros2 bag play rosbags/lap_recording_20260519_070512
```

---

### Step 6: View and Save the Plot
1. Once the bag playback completes in Terminal 2, switch back to **Terminal 3**.
2. Press **`Ctrl + C`** to stop the plotter node.
3. The script will generate the interactive plot on your screen and save it in your project folder as **`kinematics_results.png`**.
