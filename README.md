<div align="center">

# Autonomous Navigation Turtlebot4 – Mobile Robots for Critical Missions

Intelligent multi-package ROS 2 application for autonomous exploration, QR-code driven mission planning, and simulation on TurtleBot4.<br/>
Optimized for rapid experimentation in indoor structured environments (DIEM map & custom Gazebo worlds).

<br/>

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)  
[![Python 3.10](https://img.shields.io/badge/Python-3.10-green.svg)](https://www.python.org/)  
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache%202.0-orange.svg)](https://www.apache.org/licenses/LICENSE-2.0)  

</div>


## 🚀 What This Project Does

This repository integrates perception, exploration, mission logic and simulation tooling to let a TurtleBot4:  
- Explore an indoor map using an adaptive waypoint & corridor policy  
- Detect and decode QR codes to drive task sequencing (Discovery Action)  
- Plan global navigation goals and resolve dynamic mission logic  
- Run fully in Gazebo (Ignition) or on a real robot with Nav2  
- Fine‑tune geometric exploration primitives (circles, crossings)  
- Centralize configuration (maps, Nav2 params) for reproducibility

---

## 📹 Demo Video
https://github.com/user-attachments/assets/bb4c342b-8148-4e41-a206-4988f7ee03f8

---

## 🧱 Package Overview

Each ROS 2 package is cleanly separated:

- `planner_pkg`: Mission & global planning logic; startup launch brings up localization, Nav2 stack, RViz.  
- `discovery_pkg`: Exploration policy + QR code scanning pipeline (QReader integration).  
- `discovery_interface`: Custom ROS 2 action definition (`DiscoveryAction.action`).  
- `config_pkg`: Central Nav2 parameter sets and cleaned DIEM map assets (.pgm/.yaml).  
- `diem_gazebo`: Gazebo/Ignition simulation worlds (empty, square, custom DIEM world) + launch utilities.  
- `finetuning_pkg`: Tools / nodes for tuning crossing points & circular exploration geometry.  

---

## 🔍 Architecture (High-Level Flow)

```
				+---------------------+
				|   discovery_pkg     |
				|  (QR scan & explore)|
				+----------+----------+
							  | action feedback / results (DiscoveryAction)
							  v
 +------------------------------+        +--------------------+
 |         planner_pkg          |<------>| discovery_interface |
 | (mission logic & goal queue) |        |   (action spec)    |
 +---------------+--------------+        +--------------------+
					  |
					  | publishes goals
					  v
		  +-------------------+    uses params/maps    +------------------+
		  |      Nav2 Stack   |<---------------------->|    config_pkg    |
		  +---------+---------+                        +------------------+
						|
						| tf, odom, cmd_vel
						v
				+------------+
				| TurtleBot4 |
				+------------+
						^
						| simulated physics / world
						|
				+----------------+
				|  diem_gazebo   |
				+----------------+
```

---

## 🛠 Requirements

Core:
- ROS 2 (Humble)  
- Python 3.10  
- Colcon build tool  

QR Code / Vision Stack:
- Python libs: `pyzbar`, `torch` (PyTorch), `qrdet`, `numpy`  
- System: `zbar-tools` (for low-level symbol decoding)  

Simulation (optional):
- Ignition Gazebo (ROS 2 integration)  

> Tip: Install Python deps in a virtual environment to isolate versions. In the future, a Docker container may be provided.

---

## 📦 Installation & Build

1. Clone into your ROS 2 workspace `src/` (or keep as standalone workspace).  
2. Install system dependencies (example for Debian/Ubuntu):
	```bash
	sudo apt update
	sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot4-desktop zbar-tools
	```
3. (Optional) Python extras:
	```bash
	pip install pyzbar qrdet torch numpy
	```
4. Build:
	```bash
	colcon build --symlink-install
	```
5. Source:
	```bash
	source install/setup.bash
	```

---

## ▶️ Runtime: Real or Sim

### 1. Bring Up Core Localization + Nav2 + RViz
```bash
ros2 launch planner_pkg startup_launch.py
```

### 2. Start Discovery (QR exploration policy)
```bash
ros2 launch discovery_pkg discovery_launch.py
```

### 3. Start Planner (mission/global goals)
```bash
ros2 launch planner_pkg planner_launch.py
```

### 4. (Simulation) Launch Gazebo World
DIEM custom world with initial robot pose:
```bash
ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py x:=-6.3 y:=0.0 world:=diem_map
```

### 5. Initialize Pose
In RViz2, set the 2D Pose Estimate to align AMCL; the autonomous navigation & exploration sequence begins.

---

## 🧪 Fine-Tuning Utilities
Use `finetuning_pkg` to adjust:
- Crossing hub positions  
- Exploration radius layers  
- Intermediate waypoint geometry  

This enables iterative improvement without polluting core planner logic.

---

## 🗺 Map Cleanup Rationale
The DIEM map provided in the course contained obstacle artifacts (e.g., door latch pixels) that constrained valid path planning through narrow crossings. Using GIMP, these were selectively removed to better match physical traversability verified by tests, improving exploration completeness.

---

## 🧩 Custom Action
`discovery_interface/action/DiscoveryAction.action` defines the contract between exploration and mission logic. Extendable for additional semantic tags beyond QR codes.

---

## 🔧 Development Workflow (Suggested)
```bash
colcon build --symlink-install --packages-select planner_pkg
colcon test --packages-select planner_pkg
colcon test-result --verbose
```
Use `--packages-select` to iterate quickly. Add new parameters under `config_pkg/params` & reference them in launch files.

---

## 🛡 Reliability & Extensibility Ideas
- Add behavior tree plugins for dynamic task injection  
- Plug in SLAM (e.g., slam_toolbox) for online mapping variant  
- Integrate lifecycle management for graceful resets  
- Add rosbag2 scenario capture + replay harness  

---

## 🗺 Roadmap for Future Development
- [ ] Add unit/integration tests for QR detection pipeline  
- [ ] Provide Docker dev container  
- [ ] Multi-floor or multi-zone map switching  
- [ ] Metrics node (coverage %, time-to-detect)  
- [ ] Behavior Tree XML for mission sequencing  

---

## 📜 License
Packages (e.g., `planner_pkg`, `discovery_interface`) are released under the Apache License 2.0 (see individual package `LICENSE` files). Ensure any added third‑party assets are license-compatible.

---

Happy hacking & safe navigation! 🐢

