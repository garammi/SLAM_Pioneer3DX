# EKF-Based Local SLAM Without LiDAR

This repository demonstrates an implementation of local SLAM (Simultaneous Localization and Mapping) using only GPS and ZED stereo camera data without relying on LiDAR or a global map. The goal is to perform robust localization through sensor fusion using an Extended Kalman Filter (EKF).

---

##  Project Overview

In autonomous systems, especially in GPS-denied or partially observable environments, accurate localization is a challenge. In this project, we address this by fusing:

- **Visual odometry** (from ZED camera)
- **GPS odometry** (from RTK-GPS)

We use `robot_localization`'s `ekf_localization_node` to fuse these two sources and validate the resulting `/odometry/filtered` output.

---

##  File Structure

```
ekf_local_slam/
├── data/
│   ├── gps_cleaned.csv         # Preprocessed GPS odometry (x, y, timestamp)
│   ├── zed_cleaned.csv         # Preprocessed ZED odometry (x, y, timestamp)
│   └── filtered.csv            # Output of EKF fused odometry
│
├── launch/
│   └── ekf_localization.launch # Launches ekf_localization_node with config
│
├── config/
│   └── ekf.yaml                # Configuration file for sensor fusion
│
├── scripts/
│   └── zed_gps_replay.py       # Publishes GPS and ZED odometry from CSV
│
├── README.md                   # This file
```

---

##  How to Run

1. **Prepare ROS Environment**

```bash
source /opt/ros/noetic/setup.bash
cd ekf_local_slam
```

2. **Launch EKF Node**

```bash
roslaunch launch/ekf_localization.launch
```

3. **Replay Odometry Data**

```bash
rosrun scripts zed_gps_replay.py
```

4. **Record Fused Output**

```bash
rostopic echo -p /odometry/filtered > data/filtered.csv
```

---

##  Key Parameters in `ekf.yaml`

- `odom0`: /zed/odom
- `gps0`: /gps/odom
- Relative weighting can be tuned by adjusting `odom0_config` and `gps0_config`

---

##  Notes

- This SLAM method operates purely on local odometry.
- No global map or LiDAR is used.
- Accuracy depends on time synchronization and data quality of GPS/ZED.

---

##  References

- [robot_localization package](http://wiki.ros.org/robot_localization)
- ZED Camera ROS Wrapper
- RTK-GPS Data Logger
