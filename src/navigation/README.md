# Navigation Packages Documentation

## Overview

The **Navigation Packages** implement navigation functionality using standard ROS plugins.

---

## Key Components

### 1. **Costmap 2D**

- **Purpose**:
  - Generates 2D costmaps representing the environment, used for both global and local planning.
- **Functionality**:
  - **Inflates Obstacles**:
    - Expands obstacles in the map to create a safety buffer.
    - Prevents the bot from navigating too close to obstacles.
  - Integrates seamlessly with sensor data for dynamic obstacle representation.

### 2. **NavFn Plugin** (Global Planner)

- **Path Planning Algorithm**:
  - Implements **A\* (A-star)** for global path planning.
- **Benefits**:
  - **Optimal Paths**:
    - Finds the shortest path from the start to the goal while avoiding obstacles.
  - **Heuristic-Based Efficiency**:
    - Uses a cost-efficient heuristic to speed up pathfinding in complex environments.
  - **Compatibility**:
    - Works with costmaps to adapt to changing obstacle configurations.

### 3. **TEB Local Planner** (Local Planner)

- **Algorithm**:
  - **Time Elastic Band (TEB)** optimizes the robot's local path in real-time.
- **Benefits**:
  1. **Time-Optimal Path Adjustment**:
     - Considers both time and kinematic constraints for smooth and efficient path execution.
  2. **Dynamic Obstacle Avoidance**:
     - Adapts the path dynamically as new obstacles are detected.
  3. **Smooth Path Execution**:
     - Ensures minimal jerks and smooth transitions, enhancing the bot's stability.
  4. **Tight Space Navigation**:
     - Handles narrow passages effectively by dynamically adjusting the elastic band representation.
