# 🤖 ROS2-Tic-Tac-Toe-Robotic-Arm

## Overview
A **ROS 2–based autonomous Tic-Tac-Toe system** where a robotic arm plays against a human using **computer vision, game logic, pick-and-place manipulation, and speech interaction**.
The project is fully modular, YAML-configurable, and designed for **robust debugging, calibration, and academic evaluation**.

## ✨ Features

* 🎥 **Vision-based board detection**
  * ArUco marker localization
  * Perspective correction
  * Blob filtering & piece classification
  
* 🧠 **Optimal decision making**
  * Minimax algorithm with alpha-beta pruning
  * Configurable difficulty depth
    
* 🤖 **Deterministic pick-and-place**
  * YAML-defined joint trajectories
  * Repeatable, calibration-friendly motions
    
* 🔊 **Human–Robot Interaction**
  * Text-to-speech via ROS 2 Actions
  * Turn announcements and game feedback
  
* 🛠 **Debug & calibration tools**
  * Live camera tuning
  * Vision snapshot logging
  * Manual board correction mode

## 📦 System Overview

```
Camera → Vision Node   Game Logic Node
               ↓           ↓
           Main Orchestrator
        ↙         ↓         ↘
   UI Node   Pick & Place   Sound Node
```

The **Main Orchestrator** coordinates all services and actions to ensure safe, turn-based gameplay.

## 🧱 Architecture

### Core ROS 2 Nodes

| Node                | Responsibility                        |
| ------------------- | ------------------------------------- |
| `camera_tuning`     | Camera parameter tuning & persistence |
| `vision_node`       | Board detection & state estimation    |
| `game_logic_node`   | Tic-Tac-Toe decision making           |
| `picknplace_node`   | Robotic arm motion execution          |
| `sound_node`        | Text-to-speech interaction            |
| `ui_node`           | Human input & manual override         |
| `main_orchestrator` | System coordination & game flow       |

All nodes communicate using **custom ROS 2 services, messages, and actions**.

## 🔧 Requirements

### Hardware
* 6-DOF robotic arm with servo control
* Orbbec RGB / depth camera (Dabai DCW2)
* Tic-Tac-Toe board with ArUco corner markers
* Two visually distinct game pieces

### Software
* Ubuntu + ROS 2
* Python 3
* OpenCV (with ArUco support)
* YAML configuration support

## 🚀 Installation

```bash
git clone https://github.com/kllam0612/Tic-Tac-Toe-with-Rosmaster-Robot-using-CV
cd ttt_ws
colcon build
source install/setup.bash
```

Verify interfaces:
```bash
ros2 interface list | grep ttt
```

## ▶️ Running the System

Launch all nodes using Launch File:

```bash
ros2 launch ttt_main ttt_all.launch.py
```

## 🎛 Calibration & Debugging

### Camera Calibration
* Use keyboard controls in `camera_tuning`
* Adjust exposure, gain, contrast, brightness
* Press **`s`** to save settings to `camera_config.yaml`

### Vision Debugging
Debug images are automatically saved to:

```
~/Pictures/Debug
```

Includes:
* Binary threshold output
* Cleaned blob masks
* Perspective-warped board
* Classified overlays with confidence scores

### Manual Override
If mis-detections occur:
* Enter **Manual Update Mode** via the UI node
* Correct board state interactively
* Resume gameplay safely

## 🧠 Game Logic
* Uses **Minimax with Alpha-Beta Pruning**
* Adjustable difficulty via YAML
* Stateless service design for robustness
* Supports win, loss, and draw detection

## 🗂 Configuration
All tunable parameters are stored in YAML files:
* Camera & vision thresholds
* Game logic depth
* Pick-and-place trajectories
* Audio voice & speed settings

This allows **rapid iteration without code changes**.

## 📚 References
1. [ robotic_tactoe ](https://github.com/MiguelSolisSegura/robotic_tactoe/tree/main)
2. [ TicTacToe-Robot ](https://github.com/lohxinzhi/TicTacToe-Robot)

## 📌 Notes
This project was designed with:
* **Academic evaluation** in mind
* **Clear modular separation**
* **Debug-first development**
* **Reproducible experiments**
