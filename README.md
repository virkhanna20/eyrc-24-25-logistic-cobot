# Autonomous Warehouse Logistics Robot

An **Autonomous Mobile Manipulator** that navigates a warehouse, detects packages using computer vision, and performs pick-and-place operations with a robotic arm — all without human intervention.

Built as part of the **e-Yantra Robotics Competition (eYRC) 2024-25** organized by IIT Bombay, under the Logistic coBot theme.

## Demo

The system operates end-to-end autonomously:

1. **Navigate** — the mobile base drives to a target shelf using SLAM-generated maps and Nav2 path planning
2. **Detect** — a depth camera identifies ArUco-tagged packages and estimates their 6-DOF pose
3. **Pick** — MoveIt2 plans a collision-free trajectory; the UR5 arm grasps the package
4. **Transport** — the robot drives to the delivery zone while carrying the package
5. **Place** — the arm places the package at the destination and returns to idle

## System Architecture

```
                    ┌──────────────────┐
                    │  RealSense D435  │
                    │  (RGB + Depth)   │
                    └────────┬─────────┘
                             │
                    ┌────────▼─────────┐
                    │  ArUco Detector  │──── TF: obj_<id> w.r.t. base_link
                    │  (OpenCV + TF2)  │
                    └────────┬─────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
┌───────▼───────┐   ┌───────▼───────┐   ┌───────▼───────┐
│  Nav2 Stack   │   │   MoveIt2     │   │  Gazebo Sim   │
│  (AMCL + DWB) │   │  (OMPL + KDL) │   │  (Physics)    │
└───────┬───────┘   └───────┬───────┘   └───────────────┘
        │                    │
┌───────▼────────────────────▼───────┐
│         eBOT + UR5 Arm             │
│  Differential drive, 6-DOF arm,   │
│  LiDAR, IMU, vacuum gripper       │
└────────────────────────────────────┘
```

## Tech Stack

| Layer | Technology | Purpose |
|-------|-----------|---------|
| Middleware | **ROS 2 Humble** | Pub/sub communication, TF tree, launch system |
| Simulation | **Gazebo 11** | Physics, sensors, warehouse environment |
| Navigation | **Nav2** (AMCL + DWB + Dijkstra) | Autonomous path planning and obstacle avoidance |
| Localization | **robot_localization** (EKF) | Fuses odometry + IMU for accurate pose |
| Manipulation | **MoveIt2** (OMPL + KDL) | Collision-free arm trajectory planning |
| Perception | **OpenCV** ArUco + RealSense D435 | Marker detection, depth-based 3D pose estimation |
| Attachment | Custom Gazebo plugins | Runtime link attach/detach for grasping |

## Project Structure

```
├── ebot_description/        eBOT mobile robot URDF, meshes, and Gazebo spawning
├── ebot_nav2/               Navigation config (Nav2 params, EKF, SLAM maps, RViz)
├── eyantra_warehouse/       Warehouse simulation (world files, task launchers)
├── ur_description/          UR5 robot URDF/xacro and kinematic parameters
├── ur_simulation_gazebo/    UR5 Gazebo simulation launch and control
├── ur_moveit_config/        MoveIt2 SRDF, OMPL planner, kinematics config
├── ur5_control/             ArUco detection, TF publishing, arm control scripts
├── pymoveit2/               Python MoveIt2 wrapper for trajectory planning
├── tf_broadcaster_pkg/      TF2 transform broadcasting utilities
├── linkattacher_msgs/       Custom ROS 2 service for Gazebo link attach/detach
├── realsense_gazebo_plugin/ Intel RealSense D435 camera simulation plugin
└── my_auv_sim/              AUV simulation environment (bonus module)
```

## Key Implementations

### ArUco Detection & 3D Pose Estimation (`ur5_control/`)

Custom ROS 2 node that:
- Detects 4x4_50 ArUco markers in the RGB stream
- Filters by pixel area to ignore distant markers outside arm reach
- Estimates 6-DOF pose using `cv2.aruco.estimatePoseSingleMarkers`
- Fuses with aligned depth data from RealSense (mm to m conversion)
- Applies pinhole camera model to convert pixel + depth to 3D coordinates
- Publishes `camera_link → cam_<id>` and `base_link → obj_<id>` transforms

### Autonomous Navigation (`ebot_nav2/`)

- AMCL particle filter localization (300–5000 particles)
- DWB local planner with 0.26 m/s max velocity
- Dijkstra global planner with costmap inflation
- EKF sensor fusion (odometry + IMU at 30 Hz)
- Custom waypoint navigation node with predefined warehouse locations

### UR5 Arm Control

- MoveIt2 with OMPL trajectory planning
- KDL inverse kinematics solver
- Joint trajectory controller via `gazebo_ros2_control`
- Custom Gazebo link attacher plugin for grasp simulation

## Getting Started

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11
- Nav2, MoveIt2, robot_localization, slam_toolbox

### Installation

```bash
git clone https://github.com/virkhanna20/eyrc-24-25-logistic-cobot.git

cd eyrc-24-25-logistic-cobot
chmod +x requirements.sh
./requirements.sh

# Build
cd ~/colcon_ws/src
ln -s /path/to/eyrc-24-25-logistic-cobot .
cd ~/colcon_ws
colcon build --symlink-install
source install/setup.bash
```

### Running

**Navigation only:**
```bash
ros2 launch ebot_description ebot_gazebo_launch.py    # Terminal 1
ros2 launch ebot_nav2 ebot_bringup_launch.py           # Terminal 2
```

**Perception + Arm:**
```bash
ros2 launch eyantra_warehouse task1b.launch.py                          # Terminal 1
ros2 launch ur_simulation_gazebo spawn_ur5_launch_moveit.launch.py      # Terminal 2
ros2 run ur5_control aruco_detector                                      # Terminal 3
```

**Full Autonomous Pick-and-Place:**
```bash
ros2 launch eyantra_warehouse task1c.launch.py                          # Terminal 1
ros2 launch ur_simulation_gazebo spawn_ur5_launch_moveit.launch.py      # Terminal 2
```

## Algorithms

| Algorithm | What It Does | Where |
|-----------|-------------|-------|
| **AMCL** | Particle-filter localization on known map | `ebot_nav2/params/nav2_params.yaml` |
| **EKF** | Fuses wheel odometry + IMU | `ebot_nav2/config/ekf.yaml` |
| **Dijkstra** | Global shortest-path planning | Nav2 planner server |
| **DWB** | Real-time local obstacle avoidance | Nav2 controller server |
| **OMPL** | Sampling-based arm trajectory planning | MoveIt2 via `ur_moveit_config` |
| **KDL** | Closed-form inverse kinematics for UR5 | MoveIt2 kinematics plugin |
| **ArUco + PnP** | Marker detection and 6-DOF pose estimation | `ur5_control/ur5_control/aruco_detector.py` |

## Third-Party Acknowledgements

This project integrates several open-source ROS 2 packages:

- **Universal Robots** — UR5 URDF/xacro description (BSD-3-Clause)
- **Stogl Robotics** — Gazebo simulation launch infrastructure (BSD-3-Clause)
- **pymoveit2** — Python MoveIt2 interface wrapper
- **realsense_gazebo_plugin** — Intel RealSense camera simulation
- **e-Yantra, IIT Bombay** — eBOT robot description and warehouse environment

## Author

**Parth Khanna** — [GitHub](https://github.com/virkhanna20) | [Email](mailto:parthkhannawork@gmail.com)

## License

MIT License. See [LICENSE](LICENSE) for details. Third-party packages retain their original licenses (BSD-3-Clause where noted).
