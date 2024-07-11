# How to Run

## Step 1:
  Run the gazebo_house simulation or start the node that publishes the overhead camera images to:
  - `/overhead_camera/overhead_camera1/image_raw`
  - `/overhead_camera/overhead_camera2/image_raw`
  - `/overhead_camera/overhead_camera3/image_raw`
  - `/overhead_camera/overhead_camera4/image_raw` topics.

## Step 2:
  Run the `run.py` script using:
  ```sh
  python3 -m run
  ```

## How to access the resulting occupancy grid:
    1. grid.pgm and grid.yaml file is generated in "grid" folder.
    2. the grid is also published to "/Occupancy_grid" ros2 topic and can be accessed by subscribing to it.

## Latency Test

To test the latency of execution of various modules in the project:

1. **Run the project at least once.**
2. **Execute the latency analysis script:**
   - Navigate to the `Test` folder.
   - Run the `analyze.py` file using the command:
     ```bash
     python -m analyze
     ```

Results will be displayed on the terminal.
