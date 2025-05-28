# EKF-based GPS & ZED2 Sensor Fusion for Outdoor SLAM (No LiDAR)

This repository implements a lightweight, mapless SLAM system using only GPS and ZED2 visual odometry data. It uses ROS Noetic and `robot_localization`'s EKF node to perform sensor fusion and generate a robust local trajectory estimate. The goal is to achieve reliable position estimation in outdoor environments without a global map or LiDAR, by intelligently combining complementary sensor modalities.

---

## Project Overview

### Objective
- Perform local pose estimation based solely on ZED2 and GPS data.
- Avoid reliance on external maps or LiDAR.
- Apply Extended Kalman Filter (EKF) to fuse visual odometry and GPS for trajectory smoothing and noise reduction.
- Visualize and evaluate trajectory in Google Earth via `.kml` export.

### Background
- ZED2 camera: Provides dense local odometry, but drifts over time.
- RTK GPS: Offers absolute positioning but is subject to jitter/noise.
- By aligning ZED's smooth odometry with GPS's global correctness, the system seeks to produce a reliable fused trajectory in outdoor settings like a campus.

---

## File Structure

ekf/
├── ekf.yaml # Configuration for EKF node (sensor usage, frames, etc.)
├── ekf_replay.launch # Launch file for robot_localization with parameter loading
├── zed_gps_replay.py # CSV-based odometry publisher for ZED and GPS data
├── gps_odom.csv # Cleaned GPS trajectory (UTM + offset)
├── zed_odom.csv # Cleaned and aligned ZED odometry


---

## System Architecture

[gps_odom.csv] --> /gps/odom ------
--> ekf_localization_node --> /odometry/filtered
[zed_odom.csv] --> /zed/odom ------/


- The `zed_gps_replay.py` script loads time-synchronized CSVs and publishes odometry messages at 10 Hz.
- EKF fuses `/gps/odom` and `/zed/odom` and publishes the estimated pose on `/odometry/filtered`.
- No map or loop closure is involved. This is purely a local trajectory estimation process.

---

## How to Run

1. Clone this repo and build your workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/your_id/SLAM_Pioneer3DX.git
cd ..
catkin_make
source devel/setup.bash
Launch the EKF node:


roslaunch ekf ekf_replay.launch
Run the replay node in another terminal:


rosrun ekf zed_gps_replay.py
(Optional) Log filtered results:


rostopic echo -p /odometry/filtered > filtered.csv
Convert to .kml for Google Earth:
Use Python or any online tool to convert filtered.csv (UTM) to WGS84 coordinates and save as .kml.

