# Filterer Package Documentation

## Overview

The **Filterer Package** is designed to preprocess RGB images captured by the ZED2i camera, focusing on lane detection and noise reduction. It incorporates image processing techniques like dilation, erosion, and brightness adjustment to ensure robust lane visibility under varying lighting conditions.

---

## Features

1. **Lane Detection**:

   - Filters lanes from the input RGB image for subsequent processing.
   - Applies morphological operations for noise reduction.

2. **Noise Reduction**:

   - Uses **dilation** and **erosion** techniques to clean up early-stage noise in the image.

3. **Brightness Adjustment**:
   - Converts the image to HSV format.
   - Dynamically adjusts the **V (value)** channel by clamping its average value:
     - **Brightens** images in low light conditions.
     - **Dims** overly bright images during intense daylight.

---

## Input and Output

### Input:

- **RGB Image**: Provided by the ZED2i camera.

### Output:

- **Filtered Image**: Preprocessed image with lanes highlighted and noise reduced.
