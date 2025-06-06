# Goal Calculator Documentation

## Overview

The **Goal Calculator** is a critical component designed to generate intermediate navigation goals for the bot. It ensures the bot navigates effectively between GPS waypoints and through unmapped portions of the course, enabling smooth traversal in navigation.

---

## Why a Goal Calculator is Needed

- **Insufficient GPS Points**: The course provides only a few GPS coordinates, which are insufficient for the bot to navigate the entire course.
- **Intermediate Goals**: The Goal Calculator generates intermediate goals:
  - Before the bot reaches a GPS waypoint.
  - After all GPS waypoints are visited, allowing continuous navigation.
- **Flexible Navigation**: The intermediate goals act as a general directional guide rather than precise lane-following markers.

---

## Inputs and Outputs

### Inputs:

1. **Odometry Data**:
   - Provides the bot's current position and motion details.
2. **Map Data**:
   - Supplies the global map for goal calculation.
3. **GPS Data**:
   - Used for determining final goals at specified waypoints.

### Outputs:

1. **`std_msgs` to Mapper**:
   - Signals the Mapper node to relocate the map when the bot reaches the edge.
2. **Intermediate Goals**:
   - Published to **`move_base`** to guide navigation.

---

## How It Works

### 1. **Finding the First Lane Cell**

- The algorithm performs a **Breadth-First Search (BFS)** on the global map starting from the bots position to locate the **nearest lane cell**.
- This ensures that goal calculations start from the correct region of the map.

### 2. **Finding the Last Lane Cell**

- A second BFS is performed within the lane to determine the **last lane cell**.
- **Why BFS and not Euclidean Distance?**:
  - BFS calculates distances along the lane path, accommodating U-turns and sharp curves.
  - Euclidean distance could incorrectly identify closer cells outside the path.

### 3. **Offsetting the Goal**

- After identifying the last lane cell, the algorithm offsets the goal by a predefined distance.
- This offset goal serves as an **intermediate waypoint** for navigation.
- **Why Offsetting Works**:
  - Intermediate goals need only provide a general direction for navigation, not precise lane alignment.
  - This allows flexibility and simplifies the goal generation process.

### 4. **Switching to GPS Goals**

- When the bot approaches a GPS waypoint:
  - The algorithm begins publishing the GPS coordinates as goals.
- After all GPS waypoints are reached:
  - The algorithm resumes lane-following goal calculation using the above method.

---

## Modes of Operation

1. **`gps` Mode**:

   - The calculator halts lane-following calculations.
   - Publishes only the provided GPS coordinates as final goals.

2. **`lane_follow` Mode**:

   - GPS waypoint processing is stopped.
   - The algorithm continuously calculates intermediate goals based on the lane-following method.
   - Operation stops when the bot reaches the end of the lanes.

3. **`automated` Mode**:
   - Combines both `gps` and `lane_follow` functionalities for a full-course run:
     - Navigates GPS waypoints when available.
     - Switches to lane-following calculations after all GPS points are reached or when intermediate goals are needed.
