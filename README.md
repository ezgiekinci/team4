# 🏥 Multi-Floor Autonomous Navigation (AgileX Scout 2.0)

**Course:** KON414E - Principles of Robot Autonomy (Fall 2025-2026)  
**Team:** Team 4  
**Robot:** AgileX Scout 2.0  
**Environment:** ROS2 Humble / Gazebo Fortress  

This project aims to implement autonomous mobile robot navigation within a two-story hospital simulation (**Hospital With Two Floors**). The project scope includes multi-sensor fusion (Lidar + Camera + Odometry), floor detection (Marker Detection), map switching, and autonomous driving techniques using Nav2.

## 👥 Team Members
* **Ali Taylan Dursun**
* **Ezgi Ekinci**
* **Hamdi Selim Ekşi**

## 🛠️ System Requirements
* Ubuntu 22.04 LTS
* ROS2 Humble Hawksbill
* Gazebo Classic (Gazebo 11)
* **Required Packages:** `slam_toolbox`, `navigation2`, `nav2_bringup`, `robot_localization`

## 🚀 Installation

Follow these steps to clone and build the project in your local environment:


### 1. Create a workspace (If you haven't already)
```bash
mkdir -p ~/team4_ws/src
cd ~/team4_ws/src
```

### 2. Clone the repository
```bash
# Clone the repository directly into the current directory
git clone https://github.com/TaylanDursun/scout_multifloor_navigation.git
```

### 3. Install dependencies
```bash
cd ~/team4_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build and Source
```bash
colcon build --symlink-install
source install/setup.bash
```

## 🎮 Usage
### 1. Launching the Simulation
You can use the floor argument to spawn the robot on the desired floor.
**To launch on the Ground Floor (Lobby):**

```bash
ros2 launch scout_gazebo_sim scout_hospital_floors.launch.py floor:=1
```

**To launch on the Second Floor:**
```bash
ros2 launch scout_gazebo_sim scout_hospital_floors.launch.py floor:=2
```

### 2. Mapping (SLAM)
To start the mapping mode (while the simulation is running, open a new terminal):
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True slam_params_file:=/home/$USER/scout_ws/mapper_params_online_async.yaml
```
(Note: The path to the params file may vary based on your local configuration. Please update the path if you moved the file inside the repo.)


### 3. Manual Driving (Teleoperation)
Use the keyboard to drive the robot and generate the map:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 4. Saving the Map
Once mapping is complete, use the following command to save the map (using the `use_sim_time` parameter to prevent timeouts):

**Example for Floor 1:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/scout_ws/maps/hospital_floor1 --ros-args -p use_sim_time:=true
```

**Example for Floor 2:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/scout_ws/maps/hospital_floor2 --ros-args -p use_sim_time:=true
```

## 📋 Project Roadmap

### ✅ Phase 1: Setup & Configuration (Completed)
- [x] **Simulation Environment:** Integrated AgileX Scout 2.0 with the Hospital World in Gazebo.
- [x] **Launch System:** Created parametric launch files (`scout_hospital_floors.launch.py`) for dynamic floor selection.
- [x] **System Synchronization:** Resolved TF Tree and Clock Skew issues using `use_sim_time` parameters.
- [x] **Version Control:** Established GitHub repository and team collaboration workflow.

### 🚧 Phase 2: Perception & Mapping (In Progress)
- [ ] **Sensor Fusion:** Developing a ROS2 node (`simple_laser_merger.py`) to merge Front and Rear LiDAR data into a single 360° scan.
- [ ] **Mapping Floor 1:** Generating a complete occupancy grid map for the Ground Floor using SLAM Toolbox.
- [ ] **Mapping Floor 2:** Generating a separate occupancy grid map for the Second Floor.
- [ ] **Map Post-Processing:** Saving and refining `.pgm` and `.yaml` map files for Nav2.

### 📅 Phase 3: Autonomous Navigation (Planned)
- [ ] **Marker Detection:** Implementing Aruco/AprilTag detection to identify floor transitions (Elevator/Stairs).
- [ ] **Nav2 Integration:** Configuring Navigation2 stack (Costmaps, Planners, Controllers) for the Scout robot.
- [ ] **Multi-Floor Logic:** Developing the state machine to switch maps automatically based on marker detection.
- [ ] **Final Demo:** Autonomous navigation between waypoints across different floors.

## 🔗 Resources

### 🤖 Simulation & Assets
* **[AgileX UGV Simulation](https://github.com/agilexrobotics/ugv_gazebo_sim):** Official simulation packages for the Scout 2.0 robot.
* **[Hospital World Models](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps):** Source repository for the multi-floor hospital environment used in Gazebo.

### 📚 ROS 2 Libraries
* **[Navigation2 (Nav2)](https://navigation.ros.org/):** Documentation for the navigation stack used for path planning and autonomous driving.
* **[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox):** The primary package used for mapping and localization.
* **[Robot Localization](https://docs.ros.org/en/ros2_packages/rolling/api/robot_localization/):** Package used for sensor fusion (EKF) to improve odometry.
* **[ROS 2 Humble Documentation](https://docs.ros.org/en/humble/):** Official documentation for the ROS 2 distribution used in this project.
