# Mapping Algorithm Documentation

## 1. Need for a Custom Mapping Algorithm

Standard ROS plugins for mapping were unsuitable for our needs due to:

- **Delayed and noisy sensor data**: Standard plugins struggled with the inherent lag and offset in data, especially when the bot was moving.
- **Offset in sensor readings**: Resulting inaccuracies required an approach to handle noise and offsets efficiently.

Thus, we developed a **probability-based mapping algorithm** using the **log-odds framework** with additional techniques and optimizations for **efficient environment mapping**.

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

## 4. Bayesian Inference Framework

The mapping algorithm uses **Bayesian inference** to compute the probability of a grid cell being occupied or free. This is achieved through **log-odds representation**, which is efficient for additive updates.

---

The mapping node for sim differs in only one aspect. We also filter the z value of the points. This is to make sure that we are only taking in points which are close to the ground.
