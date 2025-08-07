# Navigation Packages Documentation

## Overview

The **Navigation Packages** implement navigation functionality using standard ROS plugins.

---

## Parameter Tuning Approach

This section outlines the rationale behind how we tuned key parameters to suit our specific application requirements.

### Costmap 2D

When configuring the inflation radius, our goal was to ensure that the global planner consistently generates paths centered within the lane, rather than veering toward the edges. This was achieved by setting the inflation radius and increasing the cost scaling factor to create just enough buffer space between lane boundaries and obstacles. The result is a costmap where the lowest-cost path lies toward the center of the lane, encouraging the global planner to choose safer, more predictable routes.

### TEB Local Planner

We selected the TEB local planner for its high performance in cluttered, obstacle filled environments. Beyond the default configuration, we tuned its parameters to prioritize alignment with the global plan. Since the global plan is based on a safety oriented costmap, this alignment encourages consistency between the global and local planners, resulting in a more reliable navigation.

### Local Costmap

For the local costmap, we opted for a slightly smaller inflation radius compared to the global costmap. However, we increased the cost values near obstacles to create a sharper penalty for proximity. This approach allows the robot to maintain precise control in tighter spaces while still heavily discouraging paths that bring it too close to potential collisions.

---
