# PLICP tools for laser-scan calibration
## Overview
This is a wrapper for plicp. The package contains:
- **visualization tools**
  - *scripts/draw_iteration.py*: This script will draw the correspondence for each laser scan data.
  - *scripts/draw_result.py*: This script will draw the trajectory of the robot based on the plicp results.
- **plicp_wrapper**
  - *plicp_node.cpp*: This cpp file will subscibe message using lcm and use plicp to calculate the transformation between each frame. 

**reference**: 
- https://censi.science/research/robot-perception/plicp/
- https://github.com/AndreaCensi/csm

## Usage
#### plicp_wrapper
    cd your_ws/plicp/build
    cmake ..
    make
    lcm-logplayer-gui -l udpm://239.255.76.67:7(ip)
    ./plicp_node > result.txt

This *plicp_node* will save data in the following way:
- "ref-'timeStamp'.txt": The reference scan data for each timestamp.
- "sen-'timeStamp'.txt": The measurement scan data for each time stamp.
- "result.txt": The transformation for each iteration in plicp solver between each 2 neighbor scan. Also, the correspondence information.

### draw_iteration.py
This script displays the correspondence between laser scan data.

### draw_result.py
This script calculates the trajectory of the robot based on the result of plicp solver. Then, it compare this trajectory with the trajectory from odometry data. 