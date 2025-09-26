# TurtleBot3 Navigation with MATLAB & ROS2 (TEB + Global Planners)

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![MATLAB](https://img.shields.io/badge/MATLAB-Robotics%20System%20Toolbox-orange) ![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-lightgrey)

A **portfolio robotics project** demonstrating end-to-end navigation for a TurtleBot3 robot using:
- **Global planners**: PRM & RRT (MATLAB Robotics Toolbox)
- **Local planner**: Timed Elastic Band (TEB)
- **Simulation**: Gazebo + RViz with ROS2 Humble
- **Integration**: MATLAB ↔ ROS2 bridge for real-time control

---

## 🚀 Features
- MATLAB ROS2 node that subscribes to odometry, receives goal poses, and publishes velocity commands.
- Global path planning using PRM or RRT on a binary occupancy map.
- Path optimization using TEB (`optimizePath` + `controllerTEB`).
- Visualization in MATLAB (map, planned path, optimized trajectory).
- ROS2 launch files for Gazebo simulation and Navigation2.
- Extendable to **real TurtleBot3 robot** with ROS1 bringup.

---

## 🗂 Project Structure
```
TurtleBot3-Navigation-TEB-MATLAB-ROS2/
├─ README.md
├─ LICENSE
├─ docs/
│  ├─ overview_diagram.png
│  ├─ demo_rviz.gif
│  └─ demo_matlab.gif
├─ ros2_ws/
│  ├─ src/mobile_navigation/        # ROS2 package (bridge + launch files)
│  └─ src/additional_files/         # provided lab files (maps, URDFs)
├─ matlab/
│  ├─ ros_matlab_node.m             # main script (MATLAB ROS2 node)
│  ├─ PoseHandle.m                  # helper class
│  ├─ goalHandle2goalPose.m         # transform goal poses
│  ├─ ControllerTEB.m               # wrapper for TEB controller
│  └─ planners_demo.m               # demo of PRM/RRT only
├─ simulations/
│  ├─ gazebo_worlds/rst_lab.world
│  └─ rviz_configs/
└─ results/
   ├─ path_planning_results.png
   └─ teb_navigation.gif
```

---

## ⚙️ Requirements
- **Software**:
  - Ubuntu 22.04 LTS
  - ROS2 Humble (Navigation2, TurtleBot3 packages)
  - Gazebo (Fortress or Garden)
  - MATLAB (Robotics System Toolbox)
  - colcon CLI
- **Hardware (optional)**:
  - TurtleBot3 Burger/Waffle Pi
  - Raspberry Pi (ROS1 bringup)

---

## ▶️ How to Run

### 1. Simulation with ROS2 + Gazebo
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Launch Gazebo world
ros2 launch turtlebot3_gazebo turtlebot3_rst_lab.launch.py

# Launch Navigation2
ros2 launch turtlebot3_navigation2 navigation2_rst_lab.launch.py use_sim_time:=True
```
- In RViz: set initial pose, publish navigation goals.

### 2. MATLAB Node (Bridge)
```matlab
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
setenv('ROS_DOMAIN_ID','30')
ros_matlab_node();
```
- Subscribes to `/odom`
- Publishes to `/cmd_vel`
- Subscribes to `/goal_pose`

### 3. Real TurtleBot3 (Optional)
- Run ROS1 bringup on the robot:
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
- On laptop/PC: run MATLAB node to connect via ROS master.

---

## 📊 Results
- Global planners (PRM/RRT) generate collision-free paths.
- TEB local planner optimizes trajectory and follows it in simulation.
- MATLAB visualizes robot progress and planned vs optimized paths.

*(See `results/` for figures and GIFs.)*

---

## 🔮 Future Improvements
- Dynamic obstacle avoidance
- Multi-robot navigation
- Learning-based global planners
- ROS2-native MATLAB interfaces

---

## 📜 License
MIT License — free to use, adapt, and share.

---

## 🙌 Acknowledgements
- ROS2 Navigation2 stack
- MathWorks Robotics System Toolbox
- TurtleBot3 by ROBOTIS

