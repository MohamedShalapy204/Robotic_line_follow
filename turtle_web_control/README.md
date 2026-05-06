# TurtleControl Pro: Web Interface for ROS2

This project provides a modern, web-based controller for the ROS2 `turtlesim` example. It communicates via `rosbridge_suite` to send velocity commands to the turtle.

## Prerequisites

Ensure you have the following installed:
- ROS2 Humble
- `ros-humble-turtlesim`
- `ros-humble-rosbridge-server`

## Getting Started

Follow these steps to get everything running:

### 1. Launch Turtlesim
Open a new terminal and run the turtlesim node:
```bash
ros2 run turtlesim turtlesim_node
```

### 2. Launch ROSBridge Server
Open another terminal and launch the websocket server:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
*Note: This starts the server on port `9090` by default.*

### 3. Open the Web Interface
You can host the interface using a simple Python server. In a new terminal, navigate to the `turtle_web_control` directory and run:
```bash
cd /home/mohamed/Desktop/projects/Robotic_line_follow/turtle_web_control
python3 -m http.server 8000
```
Then, open your browser and go to: `http://localhost:8000`

## Controls
- **On-screen Buttons**: Click the arrows to move the turtle.
- **Keyboard Arrows**: Use your keyboard arrows for intuitive control.
- **Spacebar**: Emergency Stop.
- **Stop Button**: Sends 0 velocity to stop the turtle.

## Troubleshooting
- **Connection Failed**: Ensure the `rosbridge_websocket` is running and you are using the correct URL (default: `ws://localhost:9090`).
- **Commands not reaching turtle**: Verify the topic name in `app.js` matches your setup (default: `/turtle1/cmd_vel`).
