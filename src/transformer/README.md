# Transformer Package Documentation

## Overview

The **Transformer Package** manages essential transformations and timestamp corrections for sensor data.

---

## Nodes

### 1. **`lidar_tf`**

- **Purpose**:
  - Corrects timestamps for data published by the Ouster LIDAR.
  - Smoothens IMU data for consistent outputs.
- **Functionality**:
  - Subscribes to essential LIDAR topics.
  - Republishes these topics using real-time ROS timestamps (**`ros::Time::now`**).

### 2. **`zed_tf`**

- **Purpose**:
  - Integrates the ZED camera's transform tree with the base link frame.
  - Adjusts camera orientation to keep point clouds parallel to the ground.
- **Functionality**:
  - Rotates the ZED transform tree about the **pitch** and **roll** axes before connecting to the base link.
- **Benefit**:
  - Ensures that changing the camera's pitch and roll does not affect the alignment of the point cloud with the ground.

### 3. **`odom_pc`**

- **Purpose**:
  - Transforms ZED point clouds to the **odom frame** for absolute coordinate mapping.
  - Facilitates accurate mapping by ensuring all point cloud data is represented in a global reference frame.
