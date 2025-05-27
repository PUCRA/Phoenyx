<!-- 🚨 TODO: Replace with your project logo -->
<p align="center">
  <img src="resources/PHOENYX-1-logo-recortado.png" alt="Project Logo"/>
</p>


> ### **🚀 From Mars to the Lab:**
>
> 🤖 **Meet Phoenyx I**, an autonomous exploration rover inspired by **NASA’s designs** — bringing **cutting-edge robotics** from the lab to Mars-like terrains.
>
> 🧠 Powered by **AI**, **computer vision**, **SLAM**, and **ROS 2**, Phoenyx I is engineered to handle both **Mars-analogue exploration** and **complex real-world robotics challenges**.
> 
> 🏆 **Winner of Best Overall Rover & Design Excellence** at **Sener-CEA's Bot Talent competition**, it’s not just a prototype, it’s a **proven platform** for **autonomous field robotics**.

---

 ## 👀 Watch it in action 
<table>
  <tr>
    <td>
      <img src="resources/Phoenyx-I.jpg" alt="Project Logo" width="800" />
    </td>
    <td>
      <ul>
        <li><a href="https://youtube.com/shorts/iHNUQLfxfGA?feature=share">📹 Perception Task</a></li>
        <li><a href="https://youtu.be/W66J1JEbJms">📹 Control Task</a></li>
        <li><a href="https://youtu.be/kr9DZYW80oY">📹 Guided Task</a></li>
        <li><a href="https://www.instagram.com/pucra.upc/">📷 Behind the Scenes</a></li>
        <li><a href="https://www.lavanguardia.com/launi/20250515/10686074/doble-victoria-equipo-upc-competicion-diseno-programacion-robots-superar-misiones-nasa.html">📈 Article in Spanish</a></li>
        <li><a href="https://www.group.sener/noticias/la-universidad-politecnica-de-catalunya-gana-la-final-de-sener-ceas-bot-talent-el-concurso-de-robotica-de-la-fundacion-sener-y-el-comite-espanol-de-automatica/">📈 Article in Sener's web</a></li>
      </ul>
    </td>
  </tr>
</table>

---

## 📂 What you'll find in this repository?

This repository contains the full **source code**, **ROS 2 packages**, and **system configurations** for **Phoenyx I**, the award-winning autonomous rover engineered by undergraduate students at the **Polytechnic University of Catalonia**. 

Built upon the [NASA JPL Open Source Rover](https://github.com/nasa-jpl/open-source-rover), this project extends the mechanical reliability of the original platform with a robust autonomy stack, turning it into a smart explorer capable of:
- 🎯 Visual detection and classification of colored and numeric markers.
- 🌐 Real-time SLAM-based localization and navigation using LiDAR 2D.
- 🧭 Global and local path planning with obstacle avoidance.
- 🧠 Onboard decision-making and autonomous goal tracking.

Phoenyx I demonstrates how high-performance autonomy can be achieved using **accessible hardware, efficient algorithms, and a ROS 2 architecture**, serving as a scalable platform for education, research, and field robotics experimentation..

---

## 📦 Jump to:

- [🎯 Competition Challenges Overview](#🎯-objectives)
- [🛠️ Development Environment](#⚙️-development-environment)
- [📂 Repo Structure](#📁-repository-structure)
- [🧪 How to Run](#🚦-how-to-run-the-system)
- [🏁 Results & Contributors](#🏁-competition-results)

---

## 🎯 Competition Challenge Overview
(Qué es SENER-CEA's Bot Talent. Resumir brevemente y enumerar las cuatro pruebas. En el apartado de cada prueba explicar brevemente en que consiten y que se a implementado)
  
### 🔍 Perception Task (kNN)

As the rasberripy is only of 4GB of RAM, using heavy Deep Learning algorithms such as CNN was almost impossible. As a solution, it was decided to use a kNN, a clasical supervised ML algorithm in order to classify images. In this way, we achieved to reduce the computational load for de RPi.

We also used clasical computer vision methods such as morphological treatments, a deep filter, and some adjustments in the image in order to only see the number ignoring the surroundings. Moreover, a stadistic study is used for detecting the colour.

### 🔍 Control Task (LiDAR-only)
In this challenge, the robot had to **autonomously navigate narrow hallways using only 2D LiDAR**, with no predefined maps or waypoints. We addressed this with a custom ROS 2 node, `linea_media.py`, which combines **local perception** and **global goal planning** via Nav2.

The node continuously analyzes the LiDAR scan (-80º to 80º), detects the most open direction, transforms it to the global `map` frame, and sends a `PoseStamped` goal to Nav2—resulting in smooth and adaptive path planning.

Optimized for a **Raspberry Pi 4B**, the implementation uses lightweight techniques like block averaging, polar gap detection, and adaptive filtering to ensure **real-time, robust, and safe navigation**, proven both in simulation and on the competition floor.

### 🔍 Guiado Task (Aruco Localization-waypoint following)

In this challenge we had to localize our robot with aruco markers given an uknown position in the map, this is done by `brain.py` (the code with **FSM structure** that coordinates all nodes)and `localizacion_aruco.py` (a node encharged of scanning the **arucomarker** and localizes by an odom reset by virtue of manual frame transformations )

This code loads a `map` in the OSR in order to keep the robot out of the boundaries of the field.

**Highlights of the implementation:**
- 📡 Real-time goal generation using filtered 2D LiDAR data  
- 🧭 Global frame transformation with optimized TF usage  
- 🔄 Continuous state-machine loop triggered via joystick  
- 🧠 Dynamic timeout and collision-aware yaw corrections  
- ⚙️ Ultra-lightweight computation tailored for low-spec hardware  

This autonomous navigation system ran **indefinitely while power was available**, allowing the robot to adapt and respond fluidly to changes in the environment without operator intervention.

This test proved to be one of the most technically demanding—and rewarding—components of the entire competition.

---

## 🛠️ Development Environment

### System Requirements 

- **OS:** Ubuntu 22.04 LTS
- **ROS 2:** Humble Hawksbill
- **Hardware:**
  - Raspberry Pi 4B (4 GB RAM)
  - YDLidar X4
  - Orbbec AstraPro Plus RGB-D Camera
  - Adafruit BNO055 IMU
  - 6-wheel rocker-bogie base from JPL OSR (6 ruedas? mejor poner que se ha utilizado la plataforma de JPL)
  - LiPo battery 4S 5200mAh
  - Arduino for Neopixel Led control 
  - INA260n for battery state check
  - Emergency button 

### ⚠ Dependencies

- `slam_toolbox` – Real-time SLAM and map generation.
- `nav2` – Path planning and navigation stack.
- `rclpy`, `geometry_msgs`, `sensor_msgs`, `tf2_ros` – ROS core packages.
- `OpenCV`, `numpy` – Image and data processing.
- `joy`, `teleop_twist_joy` – Manual control.
- `rviz2`, `gazebo_ros` – Simulation and visualization
- `scickit-learn`- AI and image recognition

(La instalación no hace falta ponerla)
Install all dependencies via:

```bash
sudo apt update && sudo apt install\
  sudo apt install ros-humble-slam-toolbox\
  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup\
  sudo apt install ros-humble-joy\
  sudo apt install python3-opencv \
  sudo apt install python3-numpy\
  sudo apt install python3-sklearn\
```

## 📁 Repository Structure

(faltaría explicar brevemente que hay dos ramas y luego ya se pone la estructura)

### Main branch:

```bash
├── src/
    .
    ├── osr_bringup/     # Basic launch files and configuration for the OSR
    ├── percepcion/      # Image recognition, color and digit detection
    ├── guiado/          # SLAM-based localization and waypoint navigation
    ├── osr_control/     # roboclaw driver comunication and kinematics 
    ├── osr_interfaces/  # Custom mesages
    ├── phoenyx_nodes/   # Multiple nodes for diferent tasks and applications
    ├── planificador/    # Package for custom launch and yaml config.
    ├── ydlidar_ros2_driver/ #SDK for launching LiDAR 
    └── OrbbekSDK_ROS2/  #SDK for launching camera nodes  
```
(Esto no lo pondría porque debe de quedar explicado en los comentarios de arriba)
- **`osr_bringup/`**: Launch files and yaml files for configuration of the basic items for the OSR.    
- **`percepcion/`**: Includes ROS 2 nodes for visual recognition of boxes, digits and colors.  
- **`guiado/`**: Manages SLAM, ArUco marker detection, and goal sequencing.  
- **`control/`**: Core logic for hallway following using LiDAR (`linea_media.py`).  
- **`osr_control/`**: Contains `rover.py` file responsable for the rover's oddometry and kinematics, also `roboclaw_wrapper.py` a node encharged of roboclaw driver comunications within `roboclaw.py` library.  
- **`phoenyx_nodes/`**: Multiple nodes for several purposes and tests
- **`ydlidar_ros2_driver/`**: Launch files and config files for launching the real lidar 
- **`planificador/`**: Contains launch file for global and local planifiers for Nav2 and yaml file for configurating Nav2 parameters. Moreover, a custom behaviour tree is been added.
- **`OrbbeckSDK_ROS2/`**: Necessary software for launching camera node. ⚠Warning⚠: Compilation takes quite long.

### Simulation branch:

```bash
├── src/
    .
    ├── osr_bringup/     # Basic launch files and configuration for the OSR
    ├── percepcion/      # Image recognition, color and digit detection
    ├── guiado/          # SLAM-based localization and waypoint navigation
    ├── osr_control/     # roboclaw driver comunication and kinematics 
    ├── osr_interfaces/  # Custom mesages
    ├── phoenyx_nodes/   # Multiple nodes for diferent tasks and applications
    ├── planificador/    # Package for custom launch and yaml config.
    ├── ydlidar_ros2_driver/ #SDK for launching LiDAR
    ├── osr_gazebo/      #SDK for launching LiDAR
    └── OrbbekSDK_ROS2/  #SDK for launching camera nodes  
```
- **`osr_gazebo/`**: Conains the C++ controller for simulating the OSR Rover. Also, custom `.worlds` with the competition challenges.Have Fun with that ;) 


## 🚦 How to Run the System

### 🧪 Simulation 

#### For control task 
```bash
# Terminal 1 - Launch simulation world
ros2 launch osr_gazebo world.launch.py

# Terminal 2 - Launch SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Terminal 3 - Launch Nav2
ros2 launch planificador planificador_launch.py use_sim_time:=true

# Terminal 4 - Launch LiDAR-based control node
ros2 launch control linea_media.launch.py use_sim_time:=true
```
#### For guiado task
```bash
# Terminal 1 - Launch simulation world
ros2 launch osr_gazebo circuito_arucos.launch.py

# Terminal 2 - Launch SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Terminal 3 - Launch Nav2
ros2 launch planificador planificador_launch.py use_sim_time:=true

# Terminal 4 - Launch Brain
ros2 run guiado brain_guiado.py use_sim_time:=true

# Terminal 5 - Publish a true on topic /aruco_scan
ros2 topic pub --once /aruco_scan std_msgs/Bool "{data: true}"

```
## 🤖 Real Robot 
#### For percepcion task
``` bash
ros2 launch prueba_percepcion.launch.py 
```
#### For control task 
```bash
ros2 launch control control.launch.py
```
#### For guiado task 
```bash
ros2 launch guiado guiado.launch.py
```

The autonomous navigation is triggered using the joystick's **A button** **`(/joy topic)`** (you dont need to run anything or code).

⚠  **please, check the [Orbbec Camera Package](https://github.com/PUCRA/Phoenyx/tree/main/OrbbecSDK_ROS2) Readme to propperly use the Orbbec camera**


( Este apartado, Technical Highlights, se podría fusionar con el  What you'll find in this repository?)
## 🧠 Technical Highlights

- ✅ **Real-time LiDAR Navigation**: Uses 2D LiDAR to dynamically generate goals and follow the central path in corridors.  
- 🎯 **Perception-Driven Behavior**: Recognizes color-coded signs and digits to inform decision-making.  
- 🛰️ **Localization via SLAM + ArUco**: Integrates simultaneous mapping and landmark-based pose refinement.  
- ⚙️ **State Machine Architecture**: Clear transitions between behavior modules ensure robust autonomy.  
- 📈 **Fully Tuned Nav2**: Adjusted navigation parameters tailored for embedded hardware and tight-space reliability.

---

## 🏁 Competition Results

- 🥇 **First Place Overall – Bot Talent 2025**  
- 🧠 **Awarded for Best Robot Design**  
- 🛡️ Achieved zero collisions in critical navigation tests  

## 🧩 Contributors 

Developed by the **PUCRA - PHOENYX I Team** from **POLYTHECNIC UNIVERSITY OF CATALONIA**  (esta linea podría ir donde se explica el equipo en  What you'll find in this repository?, el resto lo eliminaria porque ya esta dicho)
Mentored by engineers from **SENER** as part of the Bot Talent 2025 initiative.  
Built on the [NASA JPL Open Source Rover](https://github.com/nasa-jpl/open-source-rover) foundation.

---

## 🤝  Join Us

( Se podría poner LinkdIn, Insta, YT de Pucra...)
Want to build your own rover?  
Fork this repository, explore the modules, or contribute ideas.  
Let’s build the next generation of autonomous explorers together. 🌍🤖🚀


