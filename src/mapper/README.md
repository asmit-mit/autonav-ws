# Mapping Algorithm Documentation

## 1. Need for a Custom Mapping Algorithm

Standard ROS plugins for mapping were unsuitable for our needs due to:

- **Delayed and noisy sensor data**: Standard plugins struggled with the inherent lag and offset in data, especially when the bot was moving.
- **Offset in sensor readings**: Resulting inaccuracies required an approach to handle noise and offsets efficiently.

Thus, we developed a **probability-based mapping algorithm** using the **log-odds framework** with additional techniques and optimizations for:

- **Efficient environment mapping**
- **Accurate representation of sensor data**

---

## 2. Data Input and Output

### Inputs:

- **ROS Image Filtered Mask Data**: Used to identify obstacles.
- **Pointcloud Data**: Filtered using the mask to ensure accuracy.
- **Odometry Data**: Provides position and motion information.
- **std_msg Data**: Used for map relocation and other updates.

### Output:

- **Occupancy Grid Map Data**: Publishes an accurate grid map of the environment.

---

## 3. Real-Time Visualization

Below is a GIF showcasing the mapping algorithm in action:

![Mapping Algorithm in Action](images/mapping.gif)

---

## 4. Key Components of the Algorithm

### 4.1 Using `unordered_map` for Updates

- **Problem**:
  - A naive approach updates every cell in the occupancy grid map, requiring iterations over the entire grid, which is computationally expensive.
- **Optimization**:
  - Use `unordered_map` to store:
    - **Update counts**: Tracks how many times a cell was updated during the current cycle.
    - **Log-odds values**: Maintains accumulated probabilities for updated cells.
  - Only cells actively receiving updates are processed, avoiding unnecessary computations for unaltered cells.
- **Implementation**:
  - Handled in the `pointcloudCallback` function.

### 4.2 Mask-Based Filtering

- **Mechanism**:
  - Each **mask pixel** corresponds to a specific point in the point cloud.
  - Masks are used to:
    - Filter out noise.
    - Separate **obstacle** and **non-obstacle** points.
- **Benefit**:
  - Ensures that only valid and relevant data is used for mapping.
  - Removes irrelevant or erroneous data points early in the pipeline.
- **Implementation**:
  - Managed in the `pointcloudCallback` function.

### 4.3 Handling Multiple Points in a Single Cell

- **Issue**:
  - During a single update cycle, multiple point cloud data points may fall into the same grid cell.
  - This can bias the log-odds computation due to oversampling or noise.
- **Solution**:
  - Combine or average contributions from all points mapped to the same cell before updating it.
- **Benefit**:
  - Avoids disproportionate updates due to noisy or redundant data.
- **Implementation**:
  - Implemented in the `updateMap` function.

### 4.4 Prior Probability

- **Default Value**: Set to **0.5** for each cell (neutral belief).
- **Why It Matters**:
  - Ensures a balanced starting point, with no assumption of a cell being occupied or free.
  - Reflects **uncertainty in unexplored areas**:
    - Cells with no data remain at the prior probability, representing "unknown."
  - Provides a baseline for Bayesian updates:
    - Keeps sensor updates relative to the prior probability, avoiding indefinite accumulation of evidence.
- **Bayesian Formula**:
  - l_new = l_old + l_sensor - l_prior
- **Explanation of Terms**:
  - l_old : Log-odds from previous state of the cell.
  - l_sensor : Evidence from the current sensor reading.
  - l_prior : Neutral starting belief (0.5).
- **Benefit**:
  - Ensures that updates reflect relative evidence rather than absolute data, leading to robust map generation even in noisy environments.

### 4.5 Map Relocation

- **Challenge**: As the bot maps the area around it, the area might go out of bounds of the static size of the map and the need for resizing arises. Map resizing increases memory usage exponentially (4x per doubling).
- **Solution**:
  - Use a **relocate function**:
    - Re-centers the map around the bot when:
      - `std_msg` data indicates a shift.
      - Point cloud data goes out of bounds.
    - Preserves a perimeter to retain critical data during relocation.
- **Benefit**:
  - Avoids memory issues during long-term runs.
  - Maintains continuity and prevents data loss during relocation.

---

## 5. Bayesian Inference Framework

### Overview

The mapping algorithm uses **Bayesian inference** to compute the probability of a grid cell being occupied or free. This is achieved through **log-odds representation**, which is efficient for additive updates.

### Key Concepts

1. **Mathematical Consistency**:

   - Instead of maintaining raw probabilities, log-odds are used for efficiency.
   - Without subtracting the prior, repeated updates can cause the log-odds to drift and over or underestimate occupancy, especially if the same cell is updated many times.

2. **Better Long-Term Stability**:

   - Over many updates, the Bayesian method prevents probabilities from saturating too quickly to 0 or 1 unless there is overwhelming evidence.
   - The simple addition method can lead to overconfidence in the map, making it less robust to sensor noise or dynamic changes.
