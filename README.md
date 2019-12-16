# PLICP tools for laser-scan calibration
## Overview
This is a wrapper for plicp. The package contains:
- **visualization tools**
  - *scripts/draw_iteration.py*: This script will draw the correspondence for each laser scan data.
  - *scripts/draw_result.py*: This script will draw the trajectory of the robot based on the plicp results.
- **plicp_wrapper**
  - *plicp_node.cpp*: This cpp file will subscibe message using lcm and using plicp to calculate the transformation between each frame. 

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
- reference scan data for each timestamp ---->  "ref-'timeStamp'.txt"
- measurement scan data for each time stamp ----> "sen-'timeStamp'.txt"
- transformed scan data based on plicp result for each iteration ----> "result.txt"

### draw_iteration.py
