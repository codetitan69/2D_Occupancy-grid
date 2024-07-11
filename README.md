How to run:
  
  Step1:
    run the gazebo_house simulation or start the node that publishes the overhead camera images to 
      "/overhead_camera/overhead_camera1/image_raw",
      "/overhead_camera/overhead_camera2/image_raw",
      "/overhead_camera/overhead_camera3/image_raw",
      "/overhead_camera/overhead_camera4/image_raw"
    topics.

  Step2:
    Run the run.py using "python3 -m run", this script runs the other scripts in working order automatically,
    the resulting occupancy grid can be accessed by two ways:-
        1. grid.pgm and grid.yaml file is generated in "grid" folder.
        2. the grid is also published to "/Occupancy_grid" ros2 topic and can be accessed by subscribing to it.
