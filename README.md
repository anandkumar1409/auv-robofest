# AUV Project - Robofest 5.0 🌊🤖

This repository contains the software stack and ROS 2 workspace for an Autonomous Underwater Vehicle (AUV) developed for **Robofest 5.0**. 

## 🛠️ Hardware Stack
* **Flight Controller:** Pixhawk 6C running ArduSub
* **Companion Computer:** Raspberry Pi
* **Ground Control Station:** QGroundControl

## 💻 Software & Simulation Stack
This project is built using ROS 2 (Humble) and integrates several open-source submodules for simulation, control, and localization:
* **`ardupilot_gazebo`**: Plugins and models for simulating ArduPilot-based vehicles in Gazebo.
* **`bluerov2_gz`**: Gazebo simulation files specific to the BlueROV2 frame.
* **`orca4`**: Advanced ROS 2 integration and simulation packages for underwater vehicles.
* **`orb_slam_2_ros`**: Implementation of ORB-SLAM2 for ROS, used for visual odometry and mapping.
* **`ros2_shared`**: Common utilities and shared libraries for the ROS 2 nodes.
* **`gate_detector`**: Custom computer vision node for autonomous object detection and tracking.

## 🎯 Running the Simulation & Object Tracking
The vehicle is equipped with a computer vision pipeline that allows it to autonomously identify and track objects in the water. By default, the CV node is configured to track **orange** objects.

To launch the simulation and activate the tracking protocol, use the following commands:

**Terminal 1: Launch the Simulation Environment**
```bash
[cite_start]source /opt/ros/humble/setup.bash [cite: 1]
[cite_start]ros2 launch orca_bringup sim_launch.py [cite: 1]

Terminal 2: Activate the Vision Node
Bash

[cite_start]ros2 run gate_detector vision_node [cite: 1]

📂 Repository Structure

This repository represents a standard ROS 2 workspace (ros2_ws). Note that build artifacts (build/, install/, log/) are ignored to keep the repository clean.
Plaintext

auv-robofest/
├── src/
│   ├── ardupilot_gazebo/
│   ├── bluerov2_gz/
│   ├── orb_slam_2_ros/
│   ├── orca4/
│   └── ros2_shared/
└── .gitignore

🚀 Building the Workspace

To compile this code on a new Linux machine or companion computer:

    Clone the repository:
    Bash

git clone [https://github.com/anandkumar1409/auv-robofest.git](https://github.com/anandkumar1409/auv-robofest.git)
cd auv-robofest

Build the ROS 2 workspace using colcon:
Bash

colcon build --symlink-install

Source the workspace:
Bash

    source install/setup.bash

👨‍💻 Team

    Anand Kumar - Team Captain / Lead Developer
