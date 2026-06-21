# ROS 2 Hybrid Indoor/Outdoor Localization with Neuro-Fuzzy EKF

A complete ROS 2 Humble simulation stack for seamless mobile robot localization across indoor and outdoor environments. Implements the hybrid information fusion architecture of **Yousuf & Kadri (2020)** using dual Extended Kalman Filters, an online-trained Artificial Neural Network, and a Mamdani Fuzzy Logic System for adaptive sensor weighting — all orchestrated by a reactive Behavior Tree.

**Platform:** TurtleBot3 Burger &nbsp;·&nbsp; **Simulator:** Gazebo Classic &nbsp;·&nbsp; **ROS 2:** Humble

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Behavior Tree Logic](#behavior-tree-logic)
4. [Package Structure](#package-structure)
5. [Key Algorithms](#key-algorithms)
6. [ROS Topics](#ros-topics)
7. [TF Tree](#tf-tree)
8. [Prerequisites](#prerequisites)
9. [Build](#build)
10. [Run](#run)
11. [Record a Bag](#record-a-bag)
12. [Simulation Environment](#simulation-environment)
13. [Key Parameters](#key-parameters)
14. [Results](#results)
15. [References](#references)

---

## Overview

Accurate robot localization fails at the boundary between GPS-available (outdoor) and GPS-denied (indoor) environments. This project solves that transition problem with a multi-layer fusion stack:

- **Outdoors:** Two parallel EKFs (KF-1: GPS+IMU, KF-2: GPS+Odometry) run in the map frame and are blended with a fixed-weight complementary filter. The ANN trains online in the background using these fused estimates as ground truth.
- **Indoors:** GPS disappears. The ANN takes over as a GPS pseudo-sensor. A Fuzzy Logic System dynamically weights the ANN and KF-2 outputs based on detected wheel slippage.
- **Transitions:** An AABB geofence detector on `/odom` triggers seamless mode-switching with no position jumps or filter divergence.

The system completed a 131-waypoint trajectory covering all 6 rooms of a custom building plus three outdoor loops in **14.2 minutes with zero stuck events**.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                          SENSORS                             │
│   GPS (5 Hz)        IMU (100 Hz)      Wheel Odometry (50 Hz) │
└──────┬──────────────────┬───────────────────┬────────────────┘
       │                  │                   │
       ▼                  │                   │
┌─────────────────┐       │                   │
│ navsat_transform │       │                   │
│  GPS → ENU/map  │       │                   │
│ /odometry/gps   │       │                   │
└───────┬─────────┘       │                   │
        │                 │                   │
   ┌────┴────┐       ┌────┴────┐         ┌────┴────┐
   │  KF-1   │       │  KF-2   │         │  /odom  │
   │ GPS+IMU │       │GPS+Odom │         │  (raw)  │
   │/odometry│       │/odometry│         └────┬────┘
   │/global  │       │/global2 │              │
   └────┬────┘       └────┬────┘              │
        │                 │                   │
        └────────┬─────────┘                  │
                 ▼                            │
   ┌─────────────────────────┐                │
   │   complementary_filter  │                │
   │  α·KF1 + (1−α)·KF2      │                │
   │  /odometry/fused         │                │
   └────────────┬────────────┘                │
                │  (ANN training target)       │
                ▼                            │
   ┌─────────────────────────┐               │
   │  ANN (online, PyTorch)  │               │
   │  15 → 10 → 5 → 2        │               │
   │  /ann/trajectory         │               │
   └────────────┬────────────┘               │
                │                            │
                └──────────┬─────────────────┘
                           ▼
              ┌────────────────────────┐
              │      BT Brain          │  10 Hz reactive ticks
              │  ┌──────────────────┐  │
              │  │  IndoorDetector  │  │  AABB geofence on /odom
              │  │  (geofence v3)   │  │
              │  └──────────────────┘  │
              │  Fuzzy Logic weighting │
              │  TF map→odom publisher │
              │  /odometry/bt_fused    │
              └────────────┬───────────┘
                           ▼
              ┌────────────────────────┐
              │  trajectory_controller │  Waypoint follower
              │  Door guard v4         │  + stuck recovery
              │  /cmd_vel              │
              └────────────────────────┘
```

**Outdoor mode:** BT blends KF-1 and KF-2 with fixed α=0.5 → `/odometry/bt_fused`

**Indoor mode:** BT blends ANN and KF-2 with fuzzy-adaptive α → `/odometry/bt_fused`

---

## Behavior Tree Logic

The BT ticks at 10 Hz. Every tick:

```
Sequence [Main_Orchestrator]
├── Condition: CheckSensorsReady     ← block until /imu/data AND /odom are live
└── ReactiveFallback [Localization_Strategy]
    │
    ├── Sequence [GPS_Available_Branch]          ← Branch A
    │   ├── Condition: CheckGpsSignal            ← /gps/fix valid & fresh (≤2 s)
    │   ├── Action: AlignSensorFrames            ← navsat_transform live?
    │   ├── Action: EnsureKF1Active              ← /odometry/global live?
    │   ├── Action: EnsureKF2Active              ← /odometry/global2 live?
    │   ├── Action: RunComplementaryFilter_GPS   ← x = 0.5·KF1 + 0.5·KF2
    │   ├── Action: PublishFusedPosition_GPS     ← → /odometry/bt_fused
    │   └── Action: CollectAndTrainNN            ← buffer IMU+Odom+KF1+KF2; train every 5 s
    │
    └── Sequence [GPS_Unavailable_Branch]        ← Branch B (fallback)
        ├── Action: HaltKF1_FreezeKF2GPS        ← disable KF1; latch last GPS for KF2
        ├── Action: CalculateSlipError           ← e = |ω_IMU − ω_odom|
        ├── Action: FuzzifySlipError             ← μ_Low, μ_Medium, μ_High
        ├── Action: ActivateFuzzyRules           ← Mamdani 3-rule inference
        ├── Action: DefuzzifyWeights             ← singleton WA → α1 (ANN), α2 (KF2)
        └── Action: FuseNN_KF2_Adaptive         ← x = α1·ANN + α2·KF2 → /odometry/bt_fused
```

`ReactiveFallback` re-checks GPS on every tick, so the system switches between modes immediately without any state machine reset.

---

## Package Structure

```
ros2_ws/src/
├── bt_orchestrator_pkg/          C++     BT brain, IndoorDetector, TF publisher
│   ├── src/main.cpp                      All BT nodes + StateHub + IndoorDetector
│   ├── bt_xml/localization_tree.xml      BT tree definition (Groot2 compatible)
│   ├── include/.../indoor_detector.hpp   AABB geofence v3 + GPS quality scoring
│   └── launch/system.launch.py           Single-command 4-phase launch
│
├── gps_ins_pkg/                  Python  KF-1 (GPS + IMU EKF)
│   ├── config/ekf_1.yaml                 robot_localization EKF parameters
│   └── gps_ins_pkg/complementary_filter.py   Fixed-α KF1/KF2 blender
│
├── gps_odometry_pkg/             Python  KF-2 (GPS + Odometry EKF)
│   └── config/ekf_2.yaml
│
├── robot_control_brain/          Python  Online ANN (PyTorch)
│   ├── robot_control_brain/ANN.py        15→10→5→2 network, online training loop
│   └── models/nn_weights.pt              Persisted weights (auto-loaded on restart)
│
├── control_pkg/                  Python  Waypoint trajectory controller
│   ├── control_pkg/trajectory_controller.py   P-controller + door guard v4
│   └── config/waypoints.yaml             131-waypoint mission definition
│
├── robot_description_pkg/        URDF    TurtleBot3 Burger model + world
│   ├── urdf/tb3_custom.urdf.xacro
│   ├── urdf/tb3_sensors.gazebo.xacro
│   └── worlds/indoor_outdoor.world       Custom 6-room building
│
└── Tests/                                Evaluation scripts + CSV data
    └── evaluate_pose_mse.py
```

---

## Key Algorithms

### Dual EKF Backbone

Two `robot_localization` EKF instances run in the `map` frame simultaneously:

| Filter | Sensors | Output topic |
| ------ | ------- | ------------ |
| KF-1 | GPS (`/odometry/gps`) + IMU (`/imu/data`) | `/odometry/global` |
| KF-2 | GPS (`/odometry/gps`) + Wheel Odom (`/odom`) | `/odometry/global2` |

Both use Q=0.05 process noise (empirically required for Gazebo GPS σ=0.316 m) and are guarded against covariance blowup via `smooth_lagged_data` and `initial_estimate_covariance` tuning.

### Online ANN Pseudo-Sensor

Architecture: **15 → 10 → 5 → 2** (per Yousuf & Kadri Table II)

- **Activations:** log-sigmoid hidden layers, linear output
- **Input features (15):** IMU×6 (ax, ay, az, ωx, ωy, ωz) + Odometry×5 (vx, vy, ωz, x, y) + KF positions×4 (KF1.x, KF1.y, KF2.x, KF2.y)
- **Output (2):** estimated (x, y) position
- GPS is deliberately excluded — the network must generalize to GPS-denied conditions
- **EKF validity guard:** training samples are rejected when KF-1 covariance P_xx > 5.0 m²
- **Indoor anchor substitution:** last known KF positions replace live KF features when GPS is lost
- Weights persist across restarts via `models/nn_weights.pt`

### Fuzzy Logic Adaptive Weighting

Slip error: `e = |ω_IMU − ω_odom|` (rad/s)

| Membership function | Peak | Zero crossings |
| ------------------- | ---- | -------------- |
| Low | 0.00 | — / 0.10 |
| Medium | 0.15 | 0.05 / 0.25 |
| High | 0.30 | 0.20 / — |

Mamdani inference rules:

```
R1: IF slip IS Low    → α1 IS High,   α2 IS Low     (trust ANN)
R2: IF slip IS Medium → α1 IS Medium, α2 IS Medium  (split equally)
R3: IF slip IS High   → α1 IS Low,    α2 IS High    (trust KF-2)
```

Singleton weighted-average defuzzification guarantees **α1 + α2 = 1.0** at all times.

### IndoorDetector v3

- **Geofence source:** `/odom` (not `/odometry/global` — GPS bias 0.25–0.99 m causes premature firing on the global frame)
- **Method:** 2D AABB — Building X ∈ [−10.68, +11.02] m, Y ∈ [−4.00, +4.37] m
- **Hysteresis:** Schmitt-trigger with 0.20 m dead band to suppress transitions on the boundary
- **Secondary path:** GPS quality score as a fallback signal

### Door Guard v4

Crossing a doorway requires **both** conditions simultaneously:

1. Physical wall crossing (coordinate threshold in the door's normal axis)
2. Lateral alignment (within tolerance in the door's tangential axis)

7 doors × 3 passes = 21 guarded waypoints. Stuck recovery: 8 s timeout → 1.5 s backup + 1.5 s rotate-to-face.

---

## ROS Topics

| Topic | Type | Direction | Description |
| ----- | ---- | --------- | ----------- |
| `/gps/fix` | `sensor_msgs/NavSatFix` | IN | Raw GPS |
| `/imu/data` | `sensor_msgs/Imu` | IN | Raw IMU |
| `/odom` | `nav_msgs/Odometry` | IN | Wheel odometry |
| `/odometry/gps` | `nav_msgs/Odometry` | IN | GPS in ENU (navsat_transform) |
| `/odometry/global` | `nav_msgs/Odometry` | IN/OUT | KF-1 output |
| `/odometry/global2` | `nav_msgs/Odometry` | IN/OUT | KF-2 output |
| `/odometry/fused` | `nav_msgs/Odometry` | OUT | Complementary filter blend |
| `/ann/trajectory` | `nav_msgs/Odometry` | OUT | ANN position estimate |
| `/odometry/bt_fused` | `nav_msgs/Odometry` | OUT | **Final authoritative estimate** |
| `/cmd_vel` | `geometry_msgs/Twist` | OUT | Velocity commands |
| `/bt/indoor_detection` | `std_msgs/Bool` | OUT | Indoor/outdoor flag |
| `/bt/fuzzy_weights` | `std_msgs/Float32MultiArray` | OUT | Diagnostic: α1, α2 |
| `/bt/status` | `std_msgs/String` | OUT | Diagnostic: active BT branch |

---

## TF Tree

```
map
 └── odom           ← bt_brain publishes at 30 Hz (from /odometry/bt_fused)
      └── base_footprint   ← published from /odom by bt_brain
           └── base_link
                ├── imu_link
                ├── base_scan
                └── wheel_{left,right}
```

`bt_brain` publishes both `map→odom` and `odom→base_footprint` directly, bypassing `ekf_local` (unreliable in Gazebo Humble). A bootstrap identity transform is published at t=0 to unblock `navsat_transform` initialization.

---

## Prerequisites

- ROS 2 Humble
- Gazebo Classic
- [`robot_localization`](https://github.com/cra-ros-pkg/robot_localization)
- [`BehaviorTree.CPP`](https://github.com/BehaviorTree/BehaviorTree.CPP) v4
- TurtleBot3 packages
- Python 3.10+, PyTorch ≥ 2.0

```bash
sudo apt install ros-humble-robot-localization \
                 ros-humble-turtlebot3* \
                 ros-humble-nav2-msgs \
                 ros-humble-behaviortree-cpp

pip3 install torch
```

---

## Build

```bash
cd ~/ros2_neurofuzzy_ekf_localization/ros2_ws

# Full build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Rebuild only changed packages
colcon build --packages-select bt_orchestrator_pkg robot_control_brain \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

---

## Run

```bash
source ~/ros2_neurofuzzy_ekf_localization/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch bt_orchestrator_pkg system.launch.py
```

The launch is time-phased to guarantee sensor initialization order:

| Time | What starts |
| ---- | ----------- |
| t = 0 s | Gazebo + robot spawn |
| t = 5 s | Gazebo `/gazebo` publish rate set to 100 Hz |
| t = 8 s | KF-1, KF-2, navsat_transform |
| t = 9 s | BT brain (bootstraps TF tree immediately) |
| t = 12 s | complementary_filter, ANN, trajectory_controller |

To visualize in RViz:

```bash
ros2 launch bt_orchestrator_pkg system.launch.py rviz:=true
```

---

## Record a Bag

```bash
ros2 bag record \
  /odometry/global /odometry/global2 /odom /cmd_vel \
  /bt/indoor_detection /odometry/gps /gps/fix /imu/data \
  /odometry/bt_fused /odometry/fused /ann/trajectory \
  /bt/fuzzy_weights /bt/status \
  -o run_bag
```

---

## Simulation Environment

The custom `indoor_outdoor.world` contains a 6-room building surrounded by an outdoor area:

```
              NORTH (outdoor)
              ↑
┌─────────────┬────────────────┬─────────────┐
│ North-West  │  North-Center  │  North-East │
│             │ (via C2 or C1) │             │
│  [Door A2↕] │                │  [Door B2↕] │
│  [Door C2←→]│                │  [Door C1←→]│
├═════════════╪════════════════╪═════════════╡  ← Wall_1 (solid, no direct crossing)
│ South-West  │  South-Center  │  South-East │
│  [Door A←→] │ [MAIN ENTRY ↕] │  [Door B←→] │
└─────────────┴────────────────┴─────────────┘
              ↓
           SOUTH (outdoor)
```

Wall_1 (center E-W) is solid. North-Center is only reachable via SW→NW→NC or SE→NE→NC. The 131-waypoint trajectory covers all 6 rooms across **3 indoor passes** separated by **3 outdoor loops**.

---

## Key Parameters

| Parameter | Value | Notes |
| --------- | ----- | ----- |
| `Q_x / Q_y` (both EKFs) | 0.05 | Required for Gazebo GPS σ = 0.316 m |
| `linear_speed` | 0.50 m/s | |
| `angular_gain` (Kω) | 0.50 | K=0.8 caused overshoot crash |
| `max_angular_speed` | 0.50 rad/s | |
| `distance_threshold` | 0.45 m | ≥ GPS σ to avoid spurious waypoint capture |
| `angle_threshold` | 0.70 rad | +5° margin above GPS noise floor |
| `JUMP_REJECT_M` | 1.00 m | ANN oscillation spikes reach ~1.25 m |
| `GUARD_STUCK_TIMEOUT` | 8.0 s | Triggers two-phase stuck recovery |
| ANN architecture | 15→10→5→2 | Log-sigmoid hidden, linear output |
| ANN training cap | 3000 samples | Collected during Outdoor Pass 2 |
| Geofence source | `/odom` | NOT `/odometry/global` — GPS bias too variable |
| Building AABB X | [−10.68, +11.02] m | In odom frame |
| Building AABB Y | [−4.00, +4.37] m | In odom frame |
| Geofence hysteresis | 0.20 m | Schmitt-trigger dead band |

---

## Results

### Session 17 — First Clean Full Run (2026-06-01)

| Metric | Value |
| ------ | ----- |
| Waypoints completed | **131 / 131** |
| Total navigation time | **14.2 min** |
| Stuck events | **0** |
| Recovery cycles triggered | **0** |
| ANN training samples | 3000 (capped, Outdoor Pass 2) |
| Final ANN loss | 0.0000 – 0.0001 |
| KF-1 P_xx max | 0.56 m² (0% overflow) |
| KF-2 P_xx max | 0.22 m² (0% overflow) |
| GPS ENU offset (dx_gps) | −0.266 m (varies ±0.5 m run-to-run) |

All 7 door guards cleared on first approach. The south-exit guard (`s_exit`) showed an expected 2–3 cycle oscillation due to ANN lateral noise near the alignment threshold, resolving within 2 seconds without triggering stuck recovery.

Test bag recordings are in `bags/1-7/`.

---

## References

**[P1]** T. Moore and D. Stouch, *"A Generalized Extended Kalman Filter Implementation for the Robot Operating System,"* in Proc. 5th International Conference on Intelligent Systems and Applications (INTELLI), 2014.

**[P2]** S. Yousuf and M. B. Kadri, *"Information Fusion of GPS, INS and Odometer Sensors for Improving Localization Accuracy of Mobile Robots in Indoor and Outdoor Applications,"* Robotica, vol. 38, no. 9, pp. 1–27, 2020. doi:[10.1017/S0263574720000351](https://doi.org/10.1017/S0263574720000351)
