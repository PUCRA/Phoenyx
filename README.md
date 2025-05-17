<!-- 🚨 TODO: Replace with your project logo -->
<p align="center">
  <img src="docs/images/logo.png" alt="Project Logo" width="250"/>
</p>

# 🏆 Autonomous Rover – Winner of SENER-CEA Bot Talent 2025

**From Mars to the lab: a modular, perception-driven rover based on NASA’s design and engineered to excel in real-world autonomous navigation.**

---

## 🚀 Project Overview

This repository contains the complete source code and configuration for the autonomous rover that won the **SENER-CEA Bot Talent 2025** competition.

Originally based on the [NASA JPL Open Source Rover](https://github.com/nasa-jpl/open-source-rover), this project adapts the robust mechanical platform and transforms it into a fully autonomous vehicle capable of:

- Visual recognition of colors and digits.
- Continuous LiDAR-based navigation.
- SLAM localization and global path planning.
- Real-time decision making and goal tracking.

Built by students from **POLYTHECNIC UNIVERSITY OF CATALONIA**, this rover was designed with **computational efficiency**, **modular architecture**, and **robust performance** as guiding principles. It successfully passed all the competition challenges, including a final integrated mission with complex perception and navigation under time constraints.

---

## 🎯 Objectives

- Apply advanced robotics techniques in perception, planning, and control.  
- Implement a fully autonomous system that operates reliably in constrained indoor environments.  
- Validate the system both in simulation and on a real robot using only onboard processing.  
- Demonstrate the viability of autonomous systems on low-cost, open-source hardware.  

---

### 🔍 Control Task (LiDAR-only)


In this challenge, the robot had to **autonomously navigate narrow hallways using only 2D LiDAR**, with no predefined maps or waypoints. We addressed this with a custom ROS 2 node, `linea_media.py`, which combines **local perception** and **global goal planning** via Nav2.

The node continuously analyzes the LiDAR scan (-80º to 80º), detects the most open direction, transforms it to the global `map` frame, and sends a `PoseStamped` goal to Nav2—resulting in smooth and adaptive path planning.

Optimized for a **Raspberry Pi 4B**, the implementation uses lightweight techniques like block averaging, polar gap detection, and adaptive filtering to ensure **real-time, robust, and safe navigation**, proven both in simulation and on the competition floor.


**Highlights of the implementation:**
- 📡 Real-time goal generation using filtered 2D LiDAR data  
- 🧭 Global frame transformation with optimized TF usage  
- 🔄 Continuous state-machine loop triggered via joystick  
- 🧠 Dynamic timeout and collision-aware yaw corrections  
- ⚙️ Ultra-lightweight computation tailored for low-spec hardware  

This autonomous navigation system ran **indefinitely while power was available**, allowing the robot to adapt and respond fluidly to changes in the environment without operator intervention.

This test proved to be one of the most technically demanding—and rewarding—components of the entire competition.

---

## 🖼️ Media & Demonstrations

- ![Rover in action](docs/images/rover_test.jpg)
- [📹 Full Competition Run](https://youtu.be/your_video_link)
- [📷 Behind the Scenes](https://instagram.com/your_profile)

---

## ⚙️ Development Environment

### System Requirements 

- **OS:** Ubuntu 22.04 LTS
- **ROS 2:** Humble Hawksbill
- **Hardware:**
  - Raspberry Pi 4B (4 GB RAM)
  - YDLidar X4
  - Orbbec AstraPro Plus RGB-D Camera
  - Adafruit BNO055 IMU
  - 6-wheel rocker-bogie base from JPL OSR

### Dependencies <b style="color:red">⚠⚠ Hay que revisar esto ⚠⚠</b>

- `slam_toolbox` – Real-time SLAM and map generation.
- `nav2` – Path planning and navigation stack.
- `rclpy`, `geometry_msgs`, `sensor_msgs`, `tf2_ros` – ROS core packages.
- `OpenCV`, `numpy` – Image and data processing.
- `joy`, `teleop_twist_joy` – Manual control.
- `rviz2`, `gazebo_ros` – Simulation and visualization.

Install all dependencies via:

```bash
sudo apt update && sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-joy \
  python3-opencv \
  python3-numpy
```

## 📁 Repository Structure <b style="color:red">⚠⚠ Hay que revisar esto ⚠⚠</b>

```bash
.
├── osr/             # Base configuration adapted from NASA OSR
├── percepcion/      # Image recognition, color and digit detection
├── guiado/          # SLAM-based localization and waypoint navigation
├── control/         # LiDAR-based hallway navigation logic
├── global/          # Integrated mission logic for the final challenge
├── sim/             # Gazebo world and simulation configs
├── docs/            # Images, diagrams, and reports
└── nav2_params.yaml # Navigation stack configuration
```
- **`osr/`**: Contains URDFs, robot description and basic launch files.  
- **`percepcion/`**: Includes ROS 2 nodes for visual recognition of boxes, digits and colors.  
- **`guiado/`**: Manages SLAM, ArUco marker detection, and goal sequencing.  
- **`control/`**: Core logic for hallway following using LiDAR (`linea_media.py`).  
- **`global/`**: Implements the logic for the final mission combining all subsystems.  
- **`sim/`**: Launch files and environments for testing in Gazebo.  
- **`docs/`**: All diagrams, images, and evaluation media.  
- **`nav2_params.yaml`**: Parameter tuning for navigation behaviors.

## 🚦 How to Run the System

### 🧪 Simulation (Gazebo + RViz) <b style="color:red">⚠⚠ Hay que revisar esto ⚠⚠</b>

```bash
# Terminal 1 - Launch simulation world
ros2 launch sim sim_world.launch.py

# Terminal 2 - Launch SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Terminal 3 - Launch Nav2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

# Terminal 4 - Launch LiDAR-based control node
ros2 launch control linea_media.launch.py use_sim_time:=true
```

## 🤖 Real Robot <b style="color:red">⚠⚠ Hay que revisar esto ⚠⚠</b>

```bash
# Terminal 1 - Start SLAM
ros2 launch slam_toolbox online_async_launch.py

# Terminal 2 - Launch Nav2
ros2 launch nav2_bringup navigation_launch.py

# Terminal 3 - Start LiDAR navigation
ros2 run control linea_media.py

```
The autonomous navigation is triggered using the joystick's **A button** **`(/joy topic)`**.

## 🧠 Technical Highlights

- ✅ **Real-time LiDAR Navigation**: Uses 2D LiDAR to dynamically generate goals and follow the central path in corridors.  
- 🎯 **Perception-Driven Behavior**: Recognizes color-coded signs and digits to inform decision-making.  
- 🛰️ **Localization via SLAM + ArUco**: Integrates simultaneous mapping and landmark-based pose refinement.  
- ⚙️ **State Machine Architecture**: Clear transitions between behavior modules ensure robust autonomy.  
- 📈 **Fully Tuned Nav2**: Adjusted navigation parameters tailored for embedded hardware and tight-space reliability.

---

## 🏁 Competition Results <b style="color:red">⚠⚠ Hay que revisar esto ⚠⚠</b>

- 🥇 **First Place Overall – Bot Talent 2025**  
- 🧠 **Awarded for Best Robot Design**  
- 🛡️ Achieved zero collisions in critical navigation tests  

## 🧩 Contributors <b style="color:red">⚠⚠ Hay que revisar esto ⚠⚠</b>

Developed by the **PUCRA - PHOENYX I Team** from **POLYTHECNIC UNIVERSITY OF CATALONIA**  
Mentored by engineers from **SENER** as part of the Bot Talent 2025 initiative.  
Built on the [NASA JPL Open Source Rover](https://github.com/nasa-jpl/open-source-rover) foundation.

---

## 🤝 Join Us

Want to build your own rover?  
Fork this repository, explore the modules, or contribute ideas.  
Let’s build the next generation of autonomous explorers together. 🌍🤖🚀


