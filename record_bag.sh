#!/bin/bash

# This script records the essential ROS 2 topics for Phase 6 evaluation.
# Press Ctrl+C to stop recording when the robot completes two laps.

echo "Starting ROS 2 Bag Recording for Phase 6..."
echo "Recording topics: /line_error, /cmd_vel, /odom"
echo "Press Ctrl+C to stop recording after two successful laps."

# Ensure the bags directory exists
mkdir -p ./rosbags

# Record the bag with a timestamped directory
# ros2 bag record -o ./rosbags/lap_recording_$(date +%Y%m%d_%H%M%S) /line_error /cmd_vel /odom
ros2 bag record -o ./rosbags/lap_recording_$(date +%Y%m%d_%H%M%S) /odom
