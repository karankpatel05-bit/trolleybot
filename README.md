# TrolleyBot

A ROS 2 (Humble) based autonomous robotic trolley. 

## Raspberry Pi Setup and Execution

When cloning or pulling this repository onto the Raspberry Pi, you need to build the workspace locally.

### 1. Build the Workspace

```bash
cd ~/trolleybot/trolleybot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 2. Launch the TrolleyBot

Open a terminal and source the setup, then run the bringup launch file.

```bash
cd ~/trolleybot/trolleybot_ws
source install/setup.bash
ros2 launch trolleybot_bringup bringup.launch.py
```

This launch file will start:
- Robot State Publisher
- TrolleyBot Base Node (Serial communication with Arduino)
- Lidar Node (A1M8)
- SLAM Toolbox (Asynchronous Online)

You can view the map being built and control the robot via RViz from your laptop (ensure your `ROS_DOMAIN_ID` is correctly set up for both machines).
