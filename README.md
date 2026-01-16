# Multi-Robot Control Tower System

A research project implementing **scalable multi-robot fleet management** using ROS2 Domain Isolation.

## Overview

This project addresses the network scalability challenge in multi-robot systems by implementing **domain isolation** with a centralized control tower architecture.

### The Problem

In traditional ROS2 multi-robot setups, all nodes participate in **Full-Mesh DDS Discovery**, resulting in:
- **O(N²)** network overhead as robots increase
- Control tower overwhelmed by discovering every internal node of each robot
- Poor scalability for large fleets

### Our Solution

**Domain Isolation + Bridge Architecture**:
- Each robot operates in its own isolated ROS2 domain
- A lightweight bridge node relays only essential topics to the control tower
- Control tower sees **one endpoint per robot** instead of hundreds of internal nodes
- Achieves **O(N)** network overhead - linear scaling!

```
┌─────────────────────────────────────────────────────────────┐
│                    Control Tower (Domain 0)                  │
│  ┌──────────┐  ┌───────────┐  ┌─────────────────────────┐   │
│  │ React UI │──│ Rosbridge │──│ Topic Relay / Dashboard │   │
│  └──────────┘  └───────────┘  └─────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
        │                │                │
        ▼                ▼                ▼
┌───────────────┐ ┌───────────────┐ ┌───────────────┐
│ Robot 1       │ │ Robot 2       │ │ Robot 3       │
│ (Domain 1)    │ │ (Domain 2)    │ │ (Domain 3)    │
│ ┌───────────┐ │ │ ┌───────────┐ │ │ ┌───────────┐ │
│ │  Gazebo   │ │ │ │  Gazebo   │ │ │ │  Gazebo   │ │
│ │  Nav2     │ │ │ │  Nav2     │ │ │ │  Nav2     │ │
│ │  Bridge   │ │ │ │  Bridge   │ │ │ │  Bridge   │ │
│ └───────────┘ │ │ └───────────┘ │ │ └───────────┘ │
└───────────────┘ └───────────────┘ └───────────────┘
```

## Tech Stack

| Component | Technology |
|-----------|------------|
| Middleware | ROS2 Humble + CycloneDDS |
| Simulation | Gazebo Fortress (Ignition) |
| Navigation | Navigation2 |
| Robot Platform | AgileX Scout Mini |
| Frontend | React 19 + roslib |
| Environment | AWS RoboMaker Small Warehouse |

## Project Structure

```
SW_project/
├── ros2_ws/                    # Robot simulation workspace
│   └── src/scout_nav2/         # Scout Mini + Nav2 packages
├── multi_robot_ws/             # Control tower workspace
│   └── src/multi_robot_relay/  # Domain bridge nodes
├── ros2-navigation-ui/         # React web interface
├── start_robot1.sh             # Launch Robot 1 (Domain 1)
├── start_robot2.sh             # Launch Robot 2 (Domain 2)
├── start_robot3.sh             # Launch Robot 3 (Domain 3)
└── run_multi_gazebo_multi_domain.sh  # Launch all robots
```

## Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Fortress (Ignition Gazebo 6)
- Node.js 18+ (for React UI)

## Build

```bash
# Build robot simulation workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Build control tower workspace
cd ../multi_robot_ws
colcon build --symlink-install
source install/setup.bash
```

## Quick Start

### 1. Start Robots (Each in separate terminal)

```bash
# Terminal 1: Robot 1 (Domain 1)
./start_robot1.sh

# Terminal 2: Robot 2 (Domain 2)
./start_robot2.sh

# Terminal 3: Robot 3 (Domain 3)
./start_robot3.sh
```

### 2. Start Control Tower

```bash
# Terminal: Multi-Robot Control System
cd multi_robot_ws
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch multi_robot_relay multi_robot_control.launch.py
```

### 3. Start Web UI

```bash
cd ros2-navigation-ui
npm install
npm start
```

Open http://localhost:3000 in browser.

## Verify Domain Isolation

```bash
# Check topics per domain
export ROS_DOMAIN_ID=1 && ros2 topic list  # Robot 1 only
export ROS_DOMAIN_ID=2 && ros2 topic list  # Robot 2 only
export ROS_DOMAIN_ID=0 && ros2 topic list  # Control Tower
```

## Headless Mode

Run Gazebo without GUI to save CPU:

```bash
ros2 launch agilex_scout simulate_control_gazebo.launch.py gui:=false
```

## Environment Variables

```bash
export ROS_DOMAIN_ID=0                    # Control Tower domain
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # DDS implementation
export IGN_PARTITION=robot1               # Gazebo isolation
```

## Research Goals

- Network Scalability: RTPS packet generation measurement
- Resource Overhead: CPU and network usage at control tower
- Communication Determinism: Latency and jitter analysis

## License

MIT License
