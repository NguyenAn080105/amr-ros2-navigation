# 🤖 Autonomous Service Robot — Jetson Hardware Implementation

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Foxy-blue?style=for-the-badge&logo=ros&logoColor=white)
![Jetson](https://img.shields.io/badge/NVIDIA-Jetson_AGX_Xavier-76b900?style=for-the-badge&logo=nvidia&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.8+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![C++](https://img.shields.io/badge/C++-17-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![Nav2](https://img.shields.io/badge/Nav2-Foxy-informational?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)

**Deploying a full ROS 2 autonomous navigation stack — from Gazebo simulation to physical hardware on NVIDIA Jetson AGX Xavier.**

[📦 Simulation Repo](https://github.com/NguyenAn080105/mobile-robot-ros2) · [🔧 Hardware Repo](https://github.com/NguyenAn080105/mobile-robot-ros2-hardware) · [📄 Jump to Setup](#-build--run-instructions)

</div>

---

## 📌 Overview

This repository contains the **full hardware deployment** of an indoor autonomous service robot. It bridges the gap between ROS 2 simulation and a physical robot — handling everything from low-level STM32 motor communication over UART, I2C sensor fusion, and LiDAR-based localization, to high-level Nav2 autonomous navigation and checkpoint sequencing.

The robot is built around an **NVIDIA Jetson AGX Xavier** (on an **Auvidea X221-AI carrier board**) running Ubuntu 20.04 + ROS 2 Foxy. It uses a hoverboard-based differential drive controlled by an **STM32F103RCT6**, an **RPLiDAR S2E** for 2D environment sensing, a **BNO055 IMU** for orientation, and a 7-channel array of **ultrasonic sensors** for close-range safety.

> **This is not a standalone project.** It is the robotics core of a larger multi-team system — see [System Ecosystem](#-system-ecosystem--grand-project) for context.

---

## 🌐 System Ecosystem & Grand Project

This robot is the physical backbone of a multi-disciplinary service robot product built by three specialized teams:

```
╔══════════════════════════════════════════════════════════════════════╗
║                    Grand Project — System Overview                   ║
╠══════════════════════╦═══════════════════════╦═══════════════════════╣
║   App (UX/UI) Team   ║       ROS Teamm       ║        AI Team        ║
║                      ║                       ║                       ║
║  Touchscreen UI on   ║  SLAM, Localization,  ║  On-device Chatbot    ║
║  robot display:      ║  Navigation, Hardware ║  integrated into      ║
║  • Direction View    ║  integration on       ║  the App UI           ║
║  • Running View      ║  Jetson AGX Xavier    ║                       ║
║  • Chatbot UI        ║                       ║                       ║
║                      ║  Exposes REST API  →  ║                       ║
║  Sends go/stop/      ║  receives commands,   ║                       ║
║  continue/reset via  ║  publishes robot      ║                       ║
║  REST API calls      ║  state back to App    ║                       ║
╚══════════════════════╩═══════════════════════╩═══════════════════════╝
```

The App team's touchscreen UI (running directly on the robot's display) calls our ROS API to trigger navigation. When a user taps a destination, the App sends a `go <id>` command to our `navigator.py` state machine, which dispatches it to Nav2. The AI Chatbot is embedded in the same UI, allowing users to query robot status conversationally.

### 🤖 ROS Team — Full Scope

| Layer | Responsibility |
|---|---|
| **Simulation** | Gazebo world, URDF/Xacro modeling, sensor plugins, costmap validation |
| **SLAM** | `slam_toolbox` for map building; pre-built maps stored per floor |
| **Localization** | AMCL (Monte Carlo particle filter) + EKF (wheel odom + IMU fusion) |
| **Navigation** | Nav2: NavFn A* global planner, DWB local planner, BT navigator, recoveries |
| **Hardware Integration** | Sensor drivers, motor bridge, safety layer, sequenced bringup |

### 👥 Team & Acknowledgements

The SLAM, simulation, and core navigation algorithms were co-developed with my teammates:

- **[@Teammate1]** — SLAM development, slam_toolbox tuning, map management
- **[@Teammate2]** — Nav2 configuration, costmap design, behavior tree

➡️ **Main Simulation Repository:** [mobile-robot-ros2](https://github.com/NguyenAn080105/mobile-robot-ros2)

### 🔧 My Personal Contributions (This Repository)

- **Hardware Architecture** — Full system design: power topology (36V drive / 12V compute), wiring, carrier board selection (Auvidea X221-AI), connector layout
- **Jetson Environment** — ROS 2 Foxy on ARM64/JetPack, udev symlinks, FastRTPS multi-machine configuration
- **Sensor Integration** — RPLiDAR S2E (UDP/LAN), BNO055 (I2C J23), 7-channel ultrasonic array (UART/ESP32 bridge)
- **Motor Control Bridge** — Custom `wheel_odom_node.py`: STM32F103RCT6 UART packet parsing, differential drive kinematics, `cmd_vel` → STM32 FOC command encoding
- **Ultrasonic Safety Layer** — `ultrasonic_fusion_node.py`: median filter, hard-stop hysteresis gate, `/ultrasonic_scan` for Nav2 costmap integration
- **Real-World Parameter Tuning** — EKF covariance, AMCL particle filter, DWB velocity/acceleration limits, LiDAR angular exclusion zones for physical obstructions
- **Sequenced Launch System** — 13-node, 20-second hardware-safe bringup with explicit `TimerAction` delays

---

## 🔩 Hardware Architecture

### Compute Platform

| Component | Specification |
|---|---|
| **Main Computer** | NVIDIA Jetson AGX Xavier Developer Kit 16GB |
| **SoC** | NVIDIA Xavier (8-core ARM v8.2 64-bit, 512-core Volta GPU) |
| **Carrier Board** | Auvidea X221-AI |
| **OS** | Ubuntu 20.04 (JetPack) |
| **DDS Middleware** | FastRTPS / FastDDS (`rmw_fastrtps_cpp`) |
| **Dev Workflow** | VS Code Remote SSH over Wi-Fi |

### Sensors & Actuators

| Component | Model | Interface | Key Details |
|---|---|---|---|
| **LiDAR** | RPLiDAR S2E | LAN / UDP | IP `192.168.11.2`, port `8089`, Sensitivity mode |
| **IMU** | Bosch BNO055 | I2C — J23 (X221-AI) | Bus `/dev/i2c-8`, addr `0x28`, 50 Hz, NDOF fusion |
| **Motor Controller** | STM32F103RCT6 (Hoverboard) | UART — J33 → `/dev/ttyWheel` | Firmware: `hoverboard-firmware-hack-FOC` |
| **Ultrasonic Bridge** | ESP32 (7× HC-SR04) | UART → `/dev/ttyUltrasonic` | 115200 baud, `$v0,...,v6*chk` checksum frame |
| **Manual Controller** | ESP32 (separate) | UART → STM32 | Bypasses Jetson; hardware manual drive mode |
| **Drive Wheels** | Hoverboard wheels (2×) | PWM via STM32 FOC | r = 82.55 mm, wheelbase = 421.09 mm |
| **Caster Wheel** | Passive front caster | — | Mechanical support + steering stability |
| **Display** | Touchscreen | — | Runs App UI (Direction View / Running View / Chatbot) |
| **Emergency Stop** | Physical button | Hardware | Cuts 36V motor power directly |

### Power Architecture

```
Battery (36V) ──► Hoverboard Drive Electronics ──► Left Motor / Right Motor
                        │
                        └──► DC-DC Converter (12V) ──► Jetson AGX Xavier
                                                    ──► Sensors / Peripherals
```

### Full System Topology

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       NVIDIA Jetson AGX Xavier                          │
│                        (Auvidea X221-AI Board)                          │
│                                                                         │
│  ┌───────────────┐   ┌────────────────┐  ┌──────────────────────────┐   │
│  │ RPLiDAR S2E   │   │  BNO055 IMU    │  │  ESP32 Ultrasonic Bridge │   │
│  │  UDP / LAN    │   │  I2C J23 Bus8  │  │  UART /dev/ttyUltrasonic │   │
│  └──────┬────────┘   └──────┬─────────┘  └────────────┬─────────────┘   │
│         │/scan              │/imu/data                │/ultrasonic      │
│         ▼                   │                         │                 │
│  ┌─────────────────────┐    │                         │                 │
│  │    laser_filters    │    │                         │                 │
│  │  box + range +      │    │                         │                 │
│  │  angular exclusion  │    │                         │                 │
│  └────────┬────────────┘    │                         │                 │
│           │/scan_filtered   │                         │                 │
│           ▼                 ▼                         │                 │
│  ┌────────────────────────────────────────────────┐   │                 │
│  │     robot_localization — EKF Node (50 Hz)      │   │                 │
│  │  /odom (x_dot, yaw_dot) from STM32 kinematics  │   │                 │
│  │  /imu/data (yaw, yaw_dot) from BNO055          │   │                 │
│  │  → /odometry/filtered + TF odom→base_footprint │   │                 │
│  └────────────────────┬───────────────────────────┘   │                 │
│                       │                               │                 │
│  ┌────────────────────▼───────────────────────────────▼──────────────┐  │
│  │                    Nav2 Stack                                     │  │
│  │  AMCL: /scan_filtered + /map → TF map→odom                        │  │
│  │  Global Planner: NavFnPlanner (A*)                                │  │
│  │  Local Planner:  DWBLocalPlanner (20 Hz)                          │  │
│  │  Costmap sources: /scan_filtered + /ultrasonic_scan               │  │
│  └────────────────────────────┬──────────────────────────────────────┘  │
│                               │/cmd_vel_nav                             │
│  ┌────────────────────────────▼──────────────────────────────────────┐  │
│  │              ultrasonic_fusion_node  (safety gate)                │  │
│  │  hard stop ON  < 0.20 m  │  hard stop OFF > 0.30 m  (hysteresis)  │  │
│  │  → /cmd_vel (gated)  +  /safety_stop  +  /ultrasonic_scan         │  │
│  └────────────────────────────┬──────────────────────────────────────┘  │
│                               │/cmd_vel                                 │
│  ┌────────────────────────────▼──────────────────────────────────────┐  │
│  │            wheel_odom_node  (UART J33 → /dev/ttyWheel)            │  │
│  │  RX: ω_L, ω_R (rad/s) from STM32 Hall Effect sensors              │  │
│  │  TX: speed + steer commands (FOC int16 packet, 8 bytes)           │  │
│  │  Publish: /odom  via differential drive kinematics                │  │
│  └───────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
                              │ UART (J33)
               ┌──────────────▼─────────────────┐
               │      STM32F103RCT6             │
               │   hoverboard-firmware-hack-FOC │
               │   Hall Effect → ω feedback     │
               │   PID Controller → PWM         │
               └────────────────────────────────┘

  ESP32 (manual) ──UART──► STM32      (manual joystick, bypasses Jetson)
  App UI (touchscreen) ──REST API──► Jetson  (go/stop/continue/reset)
```

### Device Symlinks (udev Rules)

Stable device names are critical for reliable bringup. Configure udev on the Jetson:

```bash
# /etc/udev/rules.d/99-robot.rules

# STM32 Motor Controller (USB-to-TTL on J33)
SUBSYSTEM=="tty", ATTRS{idVendor}=="<vendor-id>", ATTRS{idProduct}=="<product-id>", SYMLINK+="ttyWheel"

# ESP32 Ultrasonic Bridge
SUBSYSTEM=="tty", ATTRS{idVendor}=="<vendor-id>", ATTRS{idProduct}=="<product-id>", SYMLINK+="ttyUltrasonic"
```

Find your device IDs before writing the rule:
```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct"
```

Apply and verify:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -la /dev/ttyWheel /dev/ttyUltrasonic
```

---

## 🗂️ Repository Structure

```
mobile-robot-ros2-hardware/
├── urdf/
│   ├── mobile_robot_v3_urdf.xacro      # Full robot description: chassis, wheels,
│   │                                   # caster, LiDAR, IMU, 8× ultrasonic links/joints
│   └── gazebo_control_v3.xacro         # Gazebo diff-drive + sensor plugins
│
├── launch/
│   └── nav_v2_launch.py                # Sequenced hardware bringup (13 nodes, 0–20s)
│
├── config/
│   ├── nav2_params_test.yaml           # AMCL, BT Navigator, DWB planner, costmaps, recovery
│   ├── ekf.yaml                        # EKF: /odom + /imu/data → /odometry/filtered
│   ├── bno055_params.yaml              # BNO055: I2C bus 8, addr 0x28, 50 Hz, NDOF mode
│   ├── wheel_odom_params.yaml          # STM32 UART: /dev/ttyWheel, 115200 baud
│   ├── laser_filter.yaml               # Box + range + angular bounds filters
│   └── checkpoints_<floor>.yaml        # Per-floor waypoint definitions (map frame)
│
├── scripts/
│   ├── wheel_odom_node.py              # STM32 UART RX→/odom pub + cmd_vel→TX bridge
│   ├── jetson_sensor_bridge.py         # ESP32 UART → 7× /ultrasonic/<name> (Range msg)
│   ├── ultrasonic_fusion_node.py       # Median filter + hard-stop gate + /ultrasonic_scan
│   ├── imu_reader.py                   # /imu/data (quaternion) → /imu/euler (Vector3)
│   ├── navigator.py                    # 7-state checkpoint FSM (go/stop/continue/reset)
│   └── checkpoint_cmd.py               # Interactive CLI for robot control
│
├── maps/
│   ├── map_e6.yaml / map_e6.pgm        # Pre-built occupancy grid: Floor E6
│   └── map_e1.yaml / map_e1.pgm        # Pre-built occupancy grid: Floor E1
│
└── meshes/
    ├── Model_v2.stl                    # Robot chassis body mesh
    ├── wheel_left.stl / wheel_right.stl
    ├── caster_wheel.stl
    └── RPLiDAR_s2.stl
```

---

## ⚙️ Software Stack

| Layer | Package / Tool | Notes |
|---|---|---|
| **OS** | Ubuntu 20.04 | ARM64, JetPack on Jetson |
| **Middleware** | ROS 2 Foxy + FastRTPS | `ROS_DOMAIN_ID=42` for multi-machine |
| **Map Building** | `slam_toolbox` | Used offline; per-floor `.pgm`+`.yaml` saved |
| **Localization** | `nav2_amcl` | Likelihood field model, 500–2000 particles |
| **Sensor Fusion** | `robot_localization` (EKF) | 50 Hz, 2D mode, `/odom` + `/imu/data` |
| **Global Planner** | `nav2_navfn_planner` (A*) | `use_astar: true`, tolerance 0.25 m |
| **Local Planner** | `dwb_core::DWBLocalPlanner` | 20 Hz controller, 7 scoring critics |
| **Behavior Tree** | `nav2_bt_navigator` | Replanning + recovery BT |
| **Recovery** | `wait`, `spin`, `backup` | Nav2 built-in recovery behaviors |
| **LiDAR Driver** | `sllidar_ros2` | UDP channel, Sensitivity scan mode |
| **IMU Driver** | `bno055` (ROS 2) | I2C, NDOF full 9-DOF fusion mode |
| **Scan Filter** | `laser_filters` | 3-stage: box + range + angular exclusion |
| **Motor Bridge** | `wheel_odom_node.py` (custom) | UART packet decode/encode, odometry pub |
| **Ultrasonic** | `ultrasonic_fusion_node.py` (custom) | Safety hard-stop gate + costmap LaserScan |

---

## 🚀 Build & Run Instructions

### 1. Prerequisites — Jetson Xavier

```bash
# ROS 2 Foxy base install (ARM64)
# Follow: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

# Core Nav2 + localization + utilities
sudo apt install -y \
  ros-foxy-nav2-bringup \
  ros-foxy-robot-localization \
  ros-foxy-laser-filters \
  ros-foxy-joint-state-publisher \
  ros-foxy-robot-state-publisher \
  ros-foxy-slam-toolbox \
  ros-foxy-tf2-ros \
  ros-foxy-tf2-tools

# BNO055 IMU driver
# Either: sudo apt install -y ros-foxy-bno055
# Or build from source:
cd ~/ros2_ws/src && git clone https://github.com/flynneva/bno055.git

# RPLiDAR S2E driver (sllidar_ros2)
cd ~/ros2_ws/src && git clone https://github.com/Slamtec/sllidar_ros2.git

# Python dependencies
pip3 install pyserial pyyaml numpy
```

### 2. Clone & Build

```bash
# Create workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/NguyenAn080105/mobile-robot-ros2-hardware.git mobile_robot

# Resolve ROS dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build (--symlink-install lets you edit Python scripts without rebuilding)
colcon build --packages-select mobile_robot --symlink-install

# Source
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 3. Environment Setup (All Machines)

Both the Jetson and any remote monitoring laptop must share the same ROS configuration:

```bash
# Add to ~/.bashrc on EVERY machine in the network
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
```

> **Network note:** Jetson and laptop must be on the same LAN/Wi-Fi subnet. ROS 2 uses DDS multicast for node discovery — no `ROS_MASTER_URI` required.

### 4. Verify Hardware Connections

Before launching, confirm all hardware is reachable:

```bash
# Serial devices
ls -la /dev/ttyWheel /dev/ttyUltrasonic

# IMU on I2C bus 8 (should show 0x28)
sudo i2cdetect -y 8

# LiDAR network reachability
ping 192.168.11.2

# UART permissions
groups $USER | grep dialout
# If missing: sudo usermod -aG dialout $USER  (then relogin)
```

### 5. Launch — Full Hardware Bringup

```bash
# Default: Floor E6
ros2 launch mobile_robot nav_v2_launch.py

# Specific floor
ros2 launch mobile_robot nav_v2_launch.py floor:=e1

# Full parameter override
ros2 launch mobile_robot nav_v2_launch.py \
    floor:=e6 \
    timeout_at_checkpoint:=30.0 \
    us_serial_port:=/dev/ttyUltrasonic \
    us_baud_rate:=115200
```

The launch sequence uses `TimerAction` delays to prevent hardware race conditions:

| Time | Node | Reason for Delay |
|---|---|---|
| `t = 0.0s` | `robot_state_publisher` | Must publish TF tree before anything else |
| `t = 0.0s` | `wheel_odom_node` | Begin reading STM32 UART immediately |
| `t = 0.0s` | `bno055` | Begin IMU data stream immediately |
| `t = 0.0s` | `sllidar_node` | Begin LiDAR UDP stream immediately |
| `t = 1.5s` | `joint_state_publisher` | Wait for URDF to be fully parsed |
| `t = 3.0s` | `imu_reader` | Wait for `/imu/data` stream to stabilize |
| `t = 3.5s` | `ultrasonic_fusion_node` | Wait for ultrasonic topics + `/robot/state` subscriber |
| `t = 7.0s` | `ekf_filter_node` | Wait for `/odom` and `/imu/data` to be stable |
| `t = 10.0s` | `scan_to_scan_filter_chain` | Wait for EKF TF (`odom → base_footprint`) |
| `t = 12.0s` | `nav2_bringup` | Wait for filtered scan + complete TF tree |
| `t = 15.0s` | Initial pose publisher | Wait for AMCL lifecycle to reach `active` |
| `t = 20.0s` | `checkpoint_navigator` | Wait for Nav2 action servers to be ready |

### 6. Visualize on Remote Laptop

```bash
# On laptop (same ROS_DOMAIN_ID=42, same subnet as Jetson)
ros2 run rviz2 rviz2

# Recommended displays to add:
# /map (Map), /scan_filtered (LaserScan), /tf (TF),
# /odometry/filtered (Odometry), /ultrasonic_scan (LaserScan)
```

### 7. Control the Robot (Interactive CLI)

```bash
ros2 run mobile_robot checkpoint_cmd.py
```

The CLI communicates via the `/robot/command` topic to the `navigator.py` state machine:

```
Checkpoint Commander v2
CMD  go <id> | stop | continue | reset | Ctrl+C exit
CP   0:Home | 1:Library | 2:Meeting Room | 3:Principal's Office | 4:Student Affairs
NAV  connected

[IDLE | go <id>]> go 1
SEND go 1 -> Library
STATE NAVIGATING           | CP -:- | NEXT stop

[NAVIGATING | stop]> stop
SEND stop
STATE STOPPED              | CP -:- | NEXT continue | reset

[STOPPED | continue | reset]> reset
SEND reset
STATE WAITING_RESET        | CP -:- | NEXT go <id> | wait home
# → robot returns home after 30s countdown if no new command
```

| Command | Valid States | Behavior |
|---|---|---|
| `go <id>` | `IDLE`, `WAITING_RESET` | Navigate to checkpoint `id` |
| `stop` | `COMPUTING_PATH`, `PRE_ROTATING`, `NAVIGATING` | Halt immediately; save current target |
| `continue` | `STOPPED` | Re-plan and resume to saved target |
| `reset` | `STOPPED` | Cancel mission; return home after 30s countdown |

### 8. Monitor Key Topics

```bash
# State machine state
ros2 topic echo /robot/state

# Human-readable status log
ros2 topic echo /robot/status_message

# Current reached checkpoint ID
ros2 topic echo /robot/current_checkpoint

# Raw wheel odometry
ros2 topic echo /odom

# EKF-fused odometry (used by Nav2)
ros2 topic echo /odometry/filtered

# Ultrasonic safety stop flag
ros2 topic echo /safety_stop

# Verify scan rates
ros2 topic hz /scan
ros2 topic hz /scan_filtered
```

---

## 🗺️ Navigation Architecture

### Sensor Data Flow

```
[RPLiDAR S2E]  ──/scan──────────► laser_filters
                                        │
                                   /scan_filtered
                                   ├──────────────► AMCL → TF map→odom
                                   └──────────────► Costmaps (global + local)

[BNO055 IMU]   ──/imu/data──┐
                             ├──► EKF (50 Hz) ──► /odometry/filtered ──► Nav2 BT
[STM32 UART]   ──/odom ─────┘                                          Nav2 DWB

                                   Nav2 DWB Local Planner
                                        │
                                   /cmd_vel_nav
                                        │
                                   ultrasonic_fusion_node
                                   (hard-stop hysteresis gate)
                                        │
                                   /cmd_vel
                                        │
                                   wheel_odom_node
                                   (UART TX → STM32 FOC)
```

### Checkpoint Navigator State Machine (`navigator.py`)

The navigator exposes a clean 4-command interface over a 7-state FSM:

```
              go <id>
   ┌──────────────────────────────────────────────────────────┐
   │                                                          │
   ▼                                                          │
┌──────┐  go <id>  ┌────────────────┐  path ok  ┌──────────────┐
│ IDLE │──────────►│ COMPUTING_PATH │──────────►│ PRE_ROTATING │
└──────┘           └────────────────┘           └──────┬───────┘
                         │                             │ converged
                    stop │                             ▼
                         │                       ┌───────────┐
                         │          stop         │ NAVIGATING│
                         │      ┌────────────────│           │
                         ▼      ▼                └─────┬─────┘
                      ┌─────────┐                      │
                      │ STOPPED │               succeeded│
                      └────┬────┘                      │
                 continue  │  reset                    ▼
                           │                        ┌──────┐
                           │                        │ IDLE │
                           ▼                        └──────┘
                   ┌──────────────┐  go <id>  ┌────────────────┐
                   │WAITING_RESET │──────────►│ COMPUTING_PATH │
                   │  30s timer   │           └────────────────┘
                   └──────┬───────┘
                          │ timeout
                          ▼
                   ┌──────────────┐
                   │RETURNING_HOME│ (max 3 retries, 5s delay each)
                   └──────────────┘
```

### Key Navigation Parameters

#### Robot Kinematics

| Parameter | Value | Source |
|---|---|---|
| Wheel radius | `0.08255 m` | `wheel_odom_params.yaml` |
| Wheel base | `0.42109 m` | `wheel_odom_params.yaml` |
| Robot footprint | `[[0.35,0.32],[0.35,-0.32],[-0.14,-0.32],[-0.14,0.32]]` | `nav2_params_test.yaml` |

#### Velocity & Acceleration (DWB Planner)

| Parameter | Value |
|---|---|
| Max linear velocity | `0.4 m/s` |
| Max angular velocity | `±1.5 rad/s` |
| Min angular speed | `0.4 rad/s` (overcomes static friction) |
| Linear acceleration | `0.1 m/s²` |
| Linear deceleration | `−0.35 m/s²` |
| Angular acceleration | `±1.0 rad/s²` |

#### Localization (AMCL + EKF)

| Parameter | Value |
|---|---|
| AMCL laser model | `likelihood_field` |
| AMCL scan topic | `/scan_filtered` |
| AMCL particle count | `500 – 2000` |
| AMCL laser range | `0.07 m – 14.9 m` |
| EKF frequency | `50 Hz` |
| EKF mode | `two_d_mode: true` (z, roll, pitch locked to 0) |
| EKF input 0 | `/odom` → `x_dot`, `yaw_dot` |
| EKF input 1 | `/imu/data` → `yaw`, `yaw_dot` |
| EKF output | `/odometry/filtered` + TF `odom → base_footprint` |

#### Costmaps

| Parameter | Local Costmap | Global Costmap |
|---|---|---|
| Reference frame | `odom` (rolling) | `map` (fixed) |
| Resolution | `0.05 m/cell` | `0.05 m/cell` |
| Size | `5 × 5 m` rolling window | Full map extent |
| Inflation radius | `0.37 m` | `0.37 m` |
| Cost scaling factor | `2.2` | `3.5` |
| Obstacle sources | `/scan_filtered` + `/ultrasonic_scan` | `/scan_filtered` |

#### Safety & Pre-Rotation

| Parameter | Value |
|---|---|
| Hard stop activate | `< 0.20 m` (any ultrasonic sensor) |
| Hard stop release | `> 0.30 m` (all sensors clear — hysteresis) |
| Pre-rotate threshold | `0.40 rad (~23°)` |
| Pre-rotate convergence | `0.05 rad (~3°)` |
| Pre-rotate P-gain | `1.5` |
| Pre-rotate max ω | `0.80 rad/s` |
| Pre-rotate min ω | `0.15 rad/s` (anti-stiction) |
| Pre-rotate timeout | `12.0 s` |

#### LiDAR Filter Pipeline

| Filter Stage | Configuration |
|---|---|
| **Box filter** | Excludes `x∈[−0.13, 0.33] m`, `y∈[−0.31, 0.31] m`, `z∈[−0.05, 1.40] m` — removes rays hitting robot body |
| **Range filter** | Valid range: `0.06 m – 15.0 m`; out-of-range → `±inf` (costmap-safe) |
| **Angular exclusion 1** | `96.7° – 107.9°` — physical obstruction on right side |
| **Angular exclusion 2** | `−111.1° – −87.8°` — physical obstruction on left side |
| **Angular exclusion 3** | `−77.8° – −21.7°` — additional structural exclusion |

---

## 🔄 Simulation → Hardware: Key Differences

| Aspect | Simulation | Hardware |
|---|---|---|
| **LiDAR** | Gazebo ray plugin → `/scan` | `sllidar_ros2` UDP driver → `/scan` |
| **IMU** | Gazebo IMU plugin → `/imu/data` | `bno055` I2C driver → `/imu/data` |
| **Odometry** | `gazebo_ros_diff_drive` ground truth | `wheel_odom_node.py` (STM32 UART kinematics) |
| **Motor commands** | Gazebo joint velocity controller | UART int16 packet → STM32 FOC PID → PWM |
| **Ultrasonic** | Simulated in Gazebo URDF | `jetson_sensor_bridge.py` + ESP32 hardware |
| **cmd_vel routing** | Direct `/cmd_vel` to Gazebo | Gated through `ultrasonic_fusion_node` safety layer |
| **TF: odom→base** | Gazebo ground truth | EKF fusion (50 Hz, odom + IMU) |
| **AMCL init** | Automatic from Gazebo spawn pose | Manual `/initialpose` publish at `t=15s` |
| **SLAM** | `slam_toolbox` live in sim | Pre-built maps loaded via `map_server` |
| **Device names** | N/A | Stable `/dev/ttyWheel`, `/dev/ttyUltrasonic` via udev |

---

## 📍 Defined Checkpoints

Checkpoints are defined in `config/checkpoints_<floor>.yaml` in the `map` frame. The `navigator.py` state machine reads these at startup and validates all IDs before accepting `go <id>` commands.

| ID | Name | x (m) | y (m) | Orientation (quaternion) |
|---|---|---|---|---|
| `0` | **Home** (charging dock / origin) | 0.000 | 0.000 | z=0.7085, w=0.7057 |
| `1` | Library | 3.978 | 1.584 | z=−0.6568, w=0.7540 |
| `2` | Meeting Room | 4.003 | −1.615 | z=0.7097, w=0.7046 |
| `3` | Principal's Office | −4.061 | 1.504 | z=−0.0125, w=0.9999 |
| `4` | Student Affairs | −4.067 | −1.580 | z=0.0228, w=0.9997 |

> Checkpoints are floor-specific. Pass `floor:=<name>` to the launch file to load the correct YAML.

---

## 📄 References & Documentation

| Resource | Link |
|---|---|
| ROS 2 Foxy | [docs.ros.org/en/foxy](https://docs.ros.org/en/foxy/) |
| Nav2 | [navigation.ros.org](https://navigation.ros.org/) |
| robot_localization (EKF) | [docs.ros.org — robot_localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) |
| BNO055 ROS 2 Driver | [github.com/flynneva/bno055](https://github.com/flynneva/bno055) |
| sllidar_ros2 (RPLiDAR) | [github.com/Slamtec/sllidar_ros2](https://github.com/Slamtec/sllidar_ros2) |
| Hoverboard FOC Firmware | [github.com/EFeru/hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC) |
| Auvidea X221-AI Carrier Board | [auvidea.eu](https://auvidea.eu) |
| DWB Local Planner Config | [Nav2 DWB Docs](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html) |
| slam_toolbox | [github.com/SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) |

---

## 📜 License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

---

<div align="center">

**NguyenAn080105** · [GitHub](https://github.com/NguyenAn080105) · Computer Engineering Student

*Hardware implementation repo — Mobile Robot LiDAR Project · ROS Team*

</div>
