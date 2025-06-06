# Launcher Package Documentation

## Overview

The **Launcher Package** serves as a central hub to manage the launch and configuration of all other packages in the system. It ensures a smooth and robust startup process while providing configuration management for individual package parameters.

---

## Features

1. **Unified Launch Management**:

   - The **`sb.launch`** file is used to initialize and run all packages simultaneously.

2. **Configuration Management**:
   - Contains configuration files for essential parameters, allowing easy tuning and adjustments for various packages.

---

## Components

### 1. Launch File

- **`sb.launch`**:
  - Executes all required packages in the system.
  - Ensures inter-package dependencies are met before full operation begins.

### 2. Configuration Files

- **Filterer Parameters**:
  - Stores HSV values for lane detection and noise filtering.
  - Brightness parameter to filter during different times of the day.
- **Mapping Parameters**:
  - Includes:
    - Log-odds parameters.
    - Map size and resolution settings.
    - Probability thresholds for occupancy grids.
- **Goal Calculator Parameters**:
  - Configures:
    - Search distance for goal generation.
    - Running mode (automated/gps/lane_follow) for goal determination.

---
