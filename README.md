# 🤖 WaiterBot — Autonomous Restaurant Navigation Robot

> **Frankfurt University of Applied Sciences (Frankfurt UAS)**  
> M.Eng. Information Technology — Autonomous & Intelligent Systems  
> Author: **Jakia Sultana** | Stack: ROS 2 Humble · Gazebo Classic · Nav2 · Ubuntu 22.04

---

## 📋 Table of Contents

1. [Project Overview](#-project-overview)
2. [System Architecture](#-system-architecture)
3. [Restaurant Environment](#-restaurant-environment)
4. [Robot Design](#-robot-design)
5. [Navigation Stack](#-navigation-stack)
6. [Project Structure](#-project-structure)
7. [Requirements](#-requirements)
8. [Installation & Setup](#-installation--setup)
9. [Running the Project](#-running-the-project)
10. [Expected Console Output](#-expected-console-output)
11. [Key Configuration Parameters](#-key-configuration-parameters)
12. [Bugs Fixed & Challenges Solved](#-bugs-fixed--challenges-solved)
13. [Known Limitations](#-known-limitations)
14. [Future Improvements](#-future-improvements)
15. [References](#-references)

---

## 🎯 Project Overview

**WaiterBot** is a simulated autonomous service robot designed to navigate a restaurant environment and visit customer tables without human intervention. The project demonstrates core autonomous mobile robotics concepts including:

- **Simultaneous Localisation and Mapping (SLAM)** awareness via a pre-built static map
- **Autonomous path planning** using the ROS 2 Nav2 stack
- **Obstacle avoidance** using LiDAR-based costmaps
- **Sequential waypoint navigation** visiting all 6 dining tables in a defined tour order

The robot starts at a **service station** (north side of the restaurant), traverses through front and back dining rows, visits each table, and returns home — mimicking a real waiter's workflow.

```
Service Station (spawn)
        │
        ▼
  T1 → T2 → T3       (front row, left to right)
              │
        T6 ← T5 ← T4  (back row, right to left — S-shaped path)
              │
        Service Station (return)
```

---

## 🏗 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      ROS 2 Humble                           │
│                                                             │
│  ┌──────────┐    ┌──────────┐    ┌──────────────────────┐  │
│  │  Gazebo  │───▶│  LiDAR   │───▶│   AMCL Localisation  │  │
│  │ Classic  │    │  /scan   │    │  (Particle Filter)   │  │
│  └──────────┘    └──────────┘    └──────────────────────┘  │
│       │                                    │                │
│       │ /odom                              │ /tf (map→odom) │
│       ▼                                    ▼                │
│  ┌──────────────────────────────────────────────────────┐   │
│  │                   Nav2 Stack                         │   │
│  │  ┌────────────┐  ┌────────────┐  ┌───────────────┐  │   │
│  │  │  Global    │  │  Local     │  │  BT Navigator │  │   │
│  │  │  Costmap   │  │  Costmap   │  │  (Behaviour   │  │   │
│  │  │(static map)│  │(LiDAR obs) │  │   Tree)       │  │   │
│  │  └────────────┘  └────────────┘  └───────────────┘  │   │
│  │  ┌────────────┐  ┌────────────┐                      │   │
│  │  │  NavFn     │  │    DWB     │                      │   │
│  │  │  Planner   │  │ Controller │                      │   │
│  │  │(global path│  │(local ctrl)│                      │   │
│  │  └────────────┘  └────────────┘                      │   │
│  └──────────────────────────────────────────────────────┘   │
│                            │                                │
│                            │ /cmd_vel                       │
│                            ▼                                │
│                   ┌──────────────┐                          │
│                   │  WaiterBot   │                          │
│                   │  Tour Script │                          │
│                   │(waiterbot_   │                          │
│                   │  tour.py)    │                          │
│                   └──────────────┘                          │
└─────────────────────────────────────────────────────────────┘
```

### Component Roles

| Component | Role |
|-----------|------|
| **Gazebo Classic** | Physics simulation — robot movement, LiDAR sensor, collision detection |
| **AMCL** | Adaptive Monte Carlo Localisation — tells Nav2 where the robot is on the map |
| **Nav2 Global Planner (NavFn)** | Dijkstra/A* path planning on the static occupancy grid map |
| **Nav2 Local Controller (DWB)** | Dynamic Window B controller — real-time velocity commands avoiding obstacles |
| **BT Navigator** | Behaviour Tree orchestration — retries, recovery behaviours (spin, backup, wait) |
| **waiterbot_tour.py** | High-level mission script — sends `NavigateToPose` goals sequentially |

---

## 🍽 Restaurant Environment

The simulated restaurant is a **16m × 12m** enclosed space built in Gazebo Classic SDF format.

### Layout

```
┌──────────────────────────────────────────────┐
│  p_NW                                p_NE   │  ← Pillars
│                                              │
│  [Service Station spawn at (0, 3.5)]         │
│                                              │
│  [T1]         [T2]         [T3]              │  ← Front row y=1.0
│   ⊓              ⊓              ⊓            │    (chairs N & S)
│                                              │
│                 (aisle)                      │
│                                              │
│  [T4]         [T5]         [T6]              │  ← Back row y=-3.0
│   ⊓              ⊓              ⊓            │    (chairs N & S)
│                                              │
│  p_SW                                p_SE   │  ← Pillars
└──────────────────────────────────────────────┘
  West wall                           East wall
```

### World Objects

| Object | Count | Description |
|--------|-------|-------------|
| Walls | 4 | North, South, East, West (16m × 12m enclosure) |
| Dining Tables | 6 | 1.4m × 0.8m × 0.8m, brown wood colour |
| Chairs | 12 | 2 per table (north & south sides), 0.45m × 0.45m |
| Pillars | 4 | Corner pillars at (±7, ±5), radius 0.3m |
| Floor | 1 | 16m × 12m, cream/beige colour |

### Table Positions

| Table | X | Y | Row |
|-------|---|---|-----|
| T1 | -5.0 | 1.0 | Front Left |
| T2 | 0.0 | 1.0 | Front Centre |
| T3 | 5.0 | 1.0 | Front Right |
| T4 | -5.0 | -3.0 | Back Left |
| T5 | 0.0 | -3.0 | Back Centre |
| T6 | 5.0 | -3.0 | Back Right |

### Occupancy Grid Map

- **Resolution:** 0.05 m/pixel (5 cm per cell)
- **Dimensions:** 400 × 320 pixels = 20m × 16m world coverage
- **Origin:** (-10.0, -8.0, 0.0)
- **Contents:** Only walls and tables are painted into the map — chairs are intentionally excluded to give the planner free space between tables

---

## 🤖 Robot Design

The WaiterBot is a **custom differential-drive robot** built with URDF/Xacro, designed to resemble a service robot carrying a food tray.

### Physical Specifications

| Property | Value |
|----------|-------|
| Drive type | Differential drive (2 wheels + 2 casters) |
| Wheel separation | 0.38 m |
| Wheel diameter | 0.12 m |
| Robot radius | 0.18 m (Nav2 planning) |
| Body | Cylindrical with head |

### Robot Links / Components

```
base_footprint
└── base_link
    ├── body_link          (main cylindrical body)
    ├── head_link          (sensor head on top of body)
    ├── screen_link        (front display panel)
    ├── tray_pole_link     (vertical pole)
    │   └── tray_link      (food tray platform)
    │       └── tray_rim_link
    ├── left_wheel_link
    ├── right_wheel_link
    ├── caster_front_link  (passive front caster)
    ├── caster_rear_link   (passive rear caster)
    └── lidar_link         (360° LiDAR sensor)
```

### Sensors

| Sensor | Type | Topic | Range |
|--------|------|-------|-------|
| LiDAR | `libgazebo_ros_ray_sensor.so` | `/scan` | 0.15m – 8.0m |
| Odometry | `libgazebo_ros_diff_drive.so` | `/odom` | — |

### Gazebo Plugins

```xml
<!-- Differential drive — provides /cmd_vel subscriber and /odom publisher -->
<plugin name="gazebo_ros_diff_drive" filename="libgazebo_ros_diff_drive.so">
  <wheel_separation>0.38</wheel_separation>
  <wheel_diameter>0.12</wheel_diameter>
</plugin>

<!-- LiDAR — publishes /scan LaserScan messages -->
<plugin name="gazebo_ros_ray_sensor" filename="libgazebo_ros_ray_sensor.so">
  <output_type>sensor_msgs/LaserScan</output_type>
  <frame_name>lidar_link</frame_name>
</plugin>
```

---

## 🧭 Navigation Stack

### Global Planner — NavFn (Dijkstra)

Computes the shortest collision-free path from the robot's current position to the goal on the **static global costmap**. Only walls and table footprints are included in this costmap — no LiDAR data — to prevent the robot's own sensor readings from corrupting long-distance plans.

### Local Controller — DWB (Dynamic Window B)

Samples velocity commands in real time and picks the best one based on weighted critics:

| Critic | Scale | Purpose |
|--------|-------|---------|
| `RotateToGoal` | 32.0 | Rotate to face goal when close |
| `PathAlign` | 32.0 | Stay aligned to global path |
| `PathDist` | 32.0 | Stay close to global path |
| `GoalAlign` | 24.0 | Face toward goal |
| `GoalDist` | 24.0 | Move toward goal |
| `BaseObstacle` | 0.02 | Avoid obstacles (low — wide aisles) |
| `Oscillation` | — | Prevent back-and-forth oscillation |

### Localisation — AMCL

Adaptive Monte Carlo Localisation using 5000 particles. Initial pose is set automatically at spawn position `(0.0, 3.5, yaw=π)`. The robot starts facing **west** to prevent drift toward the tables during initialisation.

### Costmap Configuration

| | Global Costmap | Local Costmap |
|--|---------------|---------------|
| Frame | `map` | `odom` |
| Layers | static + inflation | obstacle + inflation |
| LiDAR data | ❌ No | ✅ Yes |
| Inflation radius | 0.20 m | 0.20 m |
| Purpose | Path planning | Obstacle avoidance |

> **Key design decision:** The global costmap deliberately excludes the LiDAR obstacle layer. When LiDAR data is added to the global costmap, the robot's own sensor returns (from nearby chairs) corrupt the global plan, making the planner think the robot's current position is blocked.

### Recovery Behaviours

When navigation fails, Nav2 automatically tries:
1. **Spin** — rotate 90° to gather new LiDAR data for AMCL
2. **Backup** — reverse 0.15m to escape obstacle inflation
3. **Wait** — pause 5 seconds then retry

---

## 📁 Project Structure

```
wb4/
├── start_waiterbot.sh              # One-click launch script
├── kill_all.sh                     # Stop all ROS 2 / Gazebo processes
└── src/
    └── restaurant_robot/
        ├── package.xml             # ROS 2 package manifest
        ├── CMakeLists.txt          # Build configuration
        ├── config/
        │   └── nav2_params.yaml    # Full Nav2 parameter configuration
        ├── launch/
        │   └── restaurant_navigation.launch.py   # Main launch file
        ├── maps/
        │   ├── restaurant_map.pgm  # Occupancy grid (400×320 px, 0.05m/px)
        │   └── restaurant_map.yaml # Map metadata (origin, resolution)
        ├── rviz/
        │   └── restaurant_nav.rviz # RViz visualisation config
        ├── scripts/
        │   └── waiterbot_tour.py   # Tour mission script
        ├── urdf/
        │   └── waiter_robot.urdf.xacro  # Robot description
        └── worlds/
            └── restaurant.world    # Gazebo SDF world file
```

---

## 📦 Requirements

### Operating System
- **Ubuntu 22.04 LTS** (Jammy Jellyfish)

### ROS 2
- **ROS 2 Humble Hawksbill** (LTS)

### Simulation
- **Gazebo Classic 11** (ships with ROS 2 Humble)

### ROS 2 Packages

```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-nav2-simple-commander \
  python3-colcon-common-extensions
```

### Python Dependencies
- `rclpy` (included with ROS 2 Humble)
- `nav2_msgs` (included with nav2)
- `geometry_msgs` (included with ROS 2)

### Hardware / VM Requirements

| Resource | Minimum | Recommended |
|----------|---------|-------------|
| RAM | 4 GB | 8 GB |
| CPU cores | 2 | 4 |
| Disk space | 5 GB | 10 GB |
| GPU | Not required | Helps Gazebo rendering |

> ⚠️ **VMware users:** The `context mismatch in svga_surface_destroy` and `GLSL link result` errors in the terminal are cosmetic VMware OpenGL warnings. They do not affect navigation functionality.

---

## 🚀 Installation & Setup

### Step 1 — Install ROS 2 Humble

Follow the official guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

```bash
# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
```

### Step 2 — Install Nav2 and Gazebo packages

```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-ros \
  python3-colcon-common-extensions
```

### Step 3 — Download the project

Download `Working_Waiter_Robot.zip` and extract it:

```bash
cd ~/Downloads
unzip -o Working_Waiter_Robot.zip
```

You will see the `wb4/` folder containing the complete project.

---

## ▶️ Running the Project

### Step 1 — Launch the simulation

Open **Terminal 1** and run:

```bash
bash ~/Downloads/wb4/start_waiterbot.sh
```

This script automatically:
1. Kills any existing ROS 2 / Gazebo processes
2. Restarts the ROS 2 daemon
3. Copies the package to `~/ros2_ws/src/`
4. Builds with `colcon build`
5. Sources the install workspace
6. Launches Gazebo + Nav2 + RViz

**Wait until you see this message in Terminal 1:**
```
[lifecycle_manager_navigation]: Managed nodes are active
```

> ⏱ This typically takes 15–30 seconds.

### Step 2 — Run the tour

Open **Terminal 2** (a new terminal window) and run:

```bash
source ~/ros2_ws/install/setup.bash
python3 ~/Downloads/wb4/src/restaurant_robot/scripts/waiterbot_tour.py
```

### Step 3 — Watch the robot navigate

The robot will:
1. Wait 15 seconds for AMCL to converge
2. Publish stop commands to cancel any Gazebo spawn drift
3. Begin the S-shaped tour sequentially

---

## 📟 Expected Console Output

### Terminal 1 (launch) — key lines

```
[amcl]: Setting pose (4.800000): 0.000 3.500 -3.142
[lifecycle_manager_navigation]: Managed nodes are active
[bt_navigator]: Begin navigating from current location (0.0, 3.5) to (-5.00, 2.50)
[controller_server]: Passing new path to controller.
[controller_server]: Passing new path to controller.
...
```

### Terminal 2 (tour script)

```
  Connecting to Nav2...
  Nav2 found — waiting 15s for AMCL to converge...
  Stopping robot drift...
  ✅  Ready

  ╔══════════════════════════════════════════╗
  ║   🤖  WAITERBOT — Frankfurt UAS          ║
  ║   Autonomous Restaurant Navigation       ║
  ╚══════════════════════════════════════════╝

  ────────────────────────────────────────────────
  Tour: Home → T1 → T2 → T3 → T6 → T5 → T4 → Home
  ────────────────────────────────────────────────

  [1/7]  ➤  Table 1  (front left)
      Target: (-5.0, 2.5)
      ⏳  Navigating...
      ✅  Arrived at Table 1  (front left)

  [2/7]  ➤  Table 2  (front centre)
      Target: (0.0, 2.5)
      ⏳  Navigating...
      ✅  Arrived at Table 2  (front centre)

  [3/7]  ➤  Table 3  (front right)
      Target: (5.0, 2.5)
      ⏳  Navigating...
      ✅  Arrived at Table 3  (front right)

  [4/7]  ➤  Table 6  (back right)
      Target: (5.0, -1.2)
      ⏳  Navigating...
      ✅  Arrived at Table 6  (back right)

  [5/7]  ➤  Table 5  (back centre)
      Target: (0.0, -1.2)
      ⏳  Navigating...
      ✅  Arrived at Table 5  (back centre)

  [6/7]  ➤  Table 4  (back left)
      Target: (-5.0, -1.2)
      ⏳  Navigating...
      ✅  Arrived at Table 4  (back left)

  [7/7]  ➤  Service Station
      Target: (0.0, 3.5)
      ⏳  Navigating...
      ✅  Returned to Service Station

  ────────────────────────────────────────────────
  TOUR SUMMARY
  ────────────────────────────────────────────────
    ✅  VISITED   Table 1  (front left)
    ✅  VISITED   Table 2  (front centre)
    ✅  VISITED   Table 3  (front right)
    ✅  VISITED   Table 6  (back right)
    ✅  VISITED   Table 5  (back centre)
    ✅  VISITED   Table 4  (back left)
    ✅  RETURNED  Service Station
  ────────────────────────────────────────────────

  Tables visited: 6/6
  🎉  Full tour completed!

  ⏱   Total: 4m 30s
```

### Custom Tour Options

```bash
# Visit a single table and return home
python3 waiterbot_tour.py 3

# Visit a custom subset of tables
python3 waiterbot_tour.py 1 3 5

# Full default tour (all 6 tables)
python3 waiterbot_tour.py
```

### Stopping Everything

```bash
# In Terminal 1: press Ctrl+C
# Or run the kill script:
bash ~/Downloads/wb4/kill_all.sh
```

---

## ⚙️ Key Configuration Parameters

All navigation parameters are in `config/nav2_params.yaml`.

### Speed

```yaml
FollowPath:
  max_vel_x: 0.50       # Forward speed (m/s) — increase for faster
  max_speed_xy: 0.50    # Must match max_vel_x
  max_vel_theta: 1.0    # Rotation speed (rad/s)

velocity_smoother:
  max_velocity: [0.5, 0.0, 1.0]   # Must match above
```

### Obstacle Clearance

```yaml
# Reduce these to let the robot pass closer to obstacles
local_costmap:
  robot_radius: 0.18          # Physical robot radius in metres
  inflation_layer:
    inflation_radius: 0.20    # Buffer zone around obstacles

global_costmap:
  robot_radius: 0.18
  inflation_layer:
    inflation_radius: 0.20    # Must match local costmap
```

### Goal Tolerance

```yaml
general_goal_checker:
  xy_goal_tolerance: 0.5    # How close robot must get to goal (metres)
  yaw_goal_tolerance: 0.5   # How close yaw must match (radians)
```

### Tour Waypoints

Edit `scripts/waiterbot_tour.py` to change approach positions:

```python
TABLES = {
    1: (-5.0,  2.5, -1.5707, "Table 1  (front left)"),
    #    ↑x    ↑y   ↑yaw     ↑label
    #  x=-5   y=2.5  facing south
}
```

> **Minimum safe approach distance:**  
> `y_approach = table_chair_y + inflation_radius + robot_radius + chair_half_width`  
> `= 1.65 + 0.20 + 0.18 + 0.225 = 2.255` → use `y ≥ 2.5` for safety

---

## 🐛 Bugs Fixed & Challenges Solved

This project involved extensive debugging across multiple iterations. Below is a summary of the major issues resolved:

### Bug 1 — `package.xml` name tag corruption
**Symptom:** Build failed with invalid package name.  
**Cause:** Bash heredoc stripped characters from `<name>` tag.  
**Fix:** Write file using Python bytes instead of shell heredoc.

### Bug 2 — Nav2 plugin names (forward slash vs double colon)
**Symptom:** Nav2 crashed on startup with plugin not found errors.  
**Cause:** ROS 2 Humble requires plugin names with `/` not `::`.  
**Fix:** `nav2_behaviors/Spin` not `nav2_behaviors::Spin`.

### Bug 3 — Robot spawning inside wall inflation zone
**Symptom:** Planner immediately failed — "robot in obstacle".  
**Cause:** Spawn position too close to north wall (y=4.5, wall at y=6.0).  
**Fix:** Moved spawn to y=3.5 giving 2.5m clearance.

### Bug 4 — LiDAR corrupting global costmap
**Symptom:** Global planner failed intermittently after robot moved.  
**Cause:** LiDAR scans filled global costmap with false obstacles around robot.  
**Fix:** Removed obstacle layer from global costmap entirely. Only local costmap uses LiDAR.

### Bug 5 — Robot spinning wildly (`use_rotate_to_heading`)
**Symptom:** Robot spun continuously then reversed.  
**Cause:** `use_rotate_to_heading: true` caused controller to try exact heading alignment before moving.  
**Fix:** Switched from RegulatedPurePursuit to DWB controller which handles AMCL drift gracefully.

### Bug 6 — `obstacle_min_range` blocking robot's own position
**Symptom:** Planner refused all goals — "start position in obstacle".  
**Cause:** Chairs 0.55m from table approach position, within inflation zone. LiDAR marking robot's own position as blocked.  
**Fix:** Added `obstacle_min_range: 0.35` to ignore very close returns, and moved approach positions to y=2.5 (0.85m clearance).

### Bug 7 — Robot drifting to Table 3 on spawn
**Symptom:** Before any goals were sent, robot rolled eastward and parked on Table 3.  
**Cause:** Gazebo differential drive plugin had residual velocity after spawn. Robot spawned facing east, so drift went toward Table 3.  
**Fix:** Robot now spawns facing west (yaw=π). Tour script publishes 20× zero-velocity `/cmd_vel` commands immediately after Nav2 ready.

### Bug 8 — "Holding spawn position" frozen for 20 minutes
**Symptom:** Script printed "Holding spawn position..." and never progressed.  
**Cause:** Goal sent to `(0.0, 3.5)` while robot was at `(0.01, 3.5)` — only 1cm away. DWB `RotateToGoal` critic attempted exact yaw alignment to `π` and looped forever.  
**Fix:** Removed the hold-position goal entirely. Stop commands are sufficient to cancel drift.

### Bug 9 — AMCL initial pose mismatch
**Symptom:** Robot navigated to wrong locations, spun randomly, planned incorrect paths.  
**Cause:** Spawn position in launch file did not match `initial_pose` in `nav2_params.yaml`.  
**Fix:** Both set to `x=0, y=3.5, yaw=π` consistently.

### Bug 10 — `FollowWaypoints` silently skipping failed goals
**Symptom:** Tour completed in 6 minutes but no tables were actually visited.  
**Cause:** `FollowWaypoints` action with `stop_on_failure: false` skips failed waypoints without blocking. All goals failed silently.  
**Fix:** Switched to `NavigateToPose` one table at a time — blocks until each goal succeeds or fails before proceeding.

---

## ⚠️ Known Limitations

| Limitation | Description |
|-----------|-------------|
| **Static map only** | The map is pre-built. If obstacles move (e.g. a person walks in), the robot cannot re-plan around them without dynamic SLAM |
| **No dynamic obstacles** | People, moving chairs, and other robots are not simulated |
| **AMCL drift** | In rare cases AMCL localisation can drift significantly if the restaurant is symmetric and LiDAR readings are ambiguous |
| **VMware performance** | On VMware virtual machines, the Real Time Factor may drop below 1.0, making navigation slower than wall-clock time |
| **No task confirmation** | The robot has no mechanism to confirm a delivery was made or that a table needs service |
| **No battery model** | Robot operates indefinitely — no power consumption simulation |
| **Chairs not in map** | Chairs are present in Gazebo but not in the occupancy grid map. The local costmap detects them via LiDAR but the global planner is unaware |

---

## 🚀 Future Improvements

### Short-Term (Technical)

- [ ] **Dynamic re-routing** — Integrate `slam_toolbox` for real-time map updates when obstacles appear
- [ ] **Better AMCL initialisation** — Use `initialpose` topic from RViz for manual pose correction when localisation drifts
- [ ] **Waypoint pause with service indicator** — Add a visual/audio signal (LED flash, speaker) when robot arrives at a table
- [ ] **Add chairs to global costmap** — Paint chairs into the PGM map for more accurate global planning
- [ ] **Lifecycle management** — Restart failed Nav2 nodes automatically instead of requiring full relaunch

### Medium-Term (Features)

- [ ] **Table call system** — ROS 2 service/action API to dynamically add table requests to the tour queue
- [ ] **Multi-robot support** — Deploy 2–3 WaiterBots with conflict-free path planning using Nav2 multi-robot extensions
- [ ] **Priority queue** — High-priority tables (e.g. urgent requests) jump the queue
- [ ] **Return-to-dock when idle** — Robot automatically returns to service station after a configurable idle timeout

### Long-Term (Research)

- [ ] **Migrate to Gazebo Harmonic** — The current version uses Gazebo Classic (end-of-life Jan 2025). Migration to Gazebo Harmonic (via `ros_gz`) is recommended
- [ ] **Deep Reinforcement Learning navigation** — Replace Nav2 with an RL policy trained in simulation for more natural movement
- [ ] **Human-aware navigation** — Integrate pedestrian models and social force model to navigate politely around people
- [ ] **Real hardware deployment** — Port to TurtleBot3 Waffle or custom hardware platform

---

## 📚 References

| Resource | URL |
|----------|-----|
| ROS 2 Humble Documentation | https://docs.ros.org/en/humble/ |
| Nav2 Documentation | https://navigation.ros.org/ |
| Gazebo Classic Documentation | https://classic.gazebosim.org/tutorials |
| Nav2 DWB Controller | https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html |
| AMCL Configuration | https://navigation.ros.org/configuration/packages/configuring-amcl.html |
| NavFn Planner | https://navigation.ros.org/configuration/packages/configuring-navfn.html |
| ROS 2 URDF Tutorial | https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html |
| Gazebo ROS Packages | https://github.com/ros-simulation/gazebo_ros_pkgs |

---

## 👩‍💻 Author

**Jakia Sultana**  
M.Eng. Information Technology  
Frankfurt University of Applied Sciences (Frankfurt UAS)  
📧 student@fra-uas.de

---

## 📄 License

This project is licensed under the **MIT License** — see `package.xml` for details.

---

*Last updated: March 2026 | ROS 2 Humble | Gazebo Classic 11 | Ubuntu 22.04*
