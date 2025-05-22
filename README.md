# 🧭 ROS 2 Project – `Humble-Sim` Branch

This repository is divided into two main branches, both developed using **ROS 2 Humble**:

---

## 🌐 Branch Structure

### 🔀 `Humble-Sim`
This is the current branch, dedicated to development, testing, and validation in **simulation environments**.  
It mainly involves tools like:

- Gazebo
- RViz
- SLAM Toolbox
- Navigation2
- Manual and autonomous control in a simulated world

### 🤖 `Humble-Real`
This branch is focused on running the system on a **real physical robot**.  
It includes real sensor drivers, hardware-specific configurations, and field-tested parameters.

---

## 🎯 Why Separate Branches?

Although both branches target **ROS 2 Humble**, they are separated for the following reasons:

- Different sensor configurations and launch files
- Hardware-specific drivers and topics
- Tuned parameters for simulation vs. real-world performance
- Independent workflows and testing environments

This separation helps maintain clean, conflict-free development for both use cases.

---

## 🛠️ Getting Started

Clone this branch for simulation work:

```bash
git clone https://github.com/PUCRA/Phoenyx.git

```