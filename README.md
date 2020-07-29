# apriltag_ros (supports AprilTag2 and AprilTag3)
## Overview
This is a ROS package for sending apriltag msgs via ROS, and also uses apriltag library from the April Lab. This package supports both **AprilTag2** and the latest **AprilTag3** (default branch).

**Author: Bruce JK Huang
Maintainer: Bruce JK Huang, bjhuang@umich.edu  
Affiliation: The Biped Lab, the University of Michigan**
This package has been tested under [ROS] Kinetic and Ubuntu 16.04.

**[Note]** This package is used for automatic extrinsic calibration between a 3D LiDAR and a camera, described in paper: **3D LiDAR Intrinsic Calibration and Automatic System for LiDAR to Camera Calibration** ([PDF](https://github.com/UMich-BipedLab/automatic_lidar_camera_calibration/blob/release_v1/AutomaticCalibration.pdf)) and used for extrinsic calibration between a 3D LiDAR and a camera, described in paper: **Improvements to Target-Based 3D LiDAR to Camera Calibration** ([PDF](https://arxiv.org/abs/1910.03126)) 

**[Issues]**
If you encounter _any_ issues, I would be happy to help. If you cannot find a related one in the existing issues, please open a new one. I will try my best to help! 

## Results Overview
<img src="https://github.com/UMich-BipedLab/AprilTag_ROS/blob/AprilTag3_ros/figure/AprilTagFig.jpg" width="960">

## Input/Output
It takes a camera image and outputs the corresponding ID, tag size, pose, inner corners, and outer corners, etc. 


## Installation
Please download the [_AprilTag_msgs_](https://github.com/UMich-BipedLab/AprilTag_msgs) and place them under a catkin workspace. Inside the package, please _make_ the AprilTag library first, or you will encounter compiling error.


## Testing Datasets
Please download from [here](https://drive.google.com/drive/folders/1MwA2dn6_U3rCWh9gxCe8OtWWgSxriVcW?usp=sharing).


## Running
This package provides a launch file that you can directly run the package.


## Citations
1. Jiunn-Kai Huang, Chenxi Feng, Madhav Achar, Maani Ghaffari, and Jessy W. Grizzle, "3D LiDAR Intrinsic Calibration and Automatic System for LiDAR to Camera Calibration" ([PDF](https://github.com/UMich-BipedLab/automatic_lidar_camera_calibration/blob/release_v1/AutomaticCalibration.pdf))(arXiv will appear soon))
<!--
```
@article{huang2019improvements,
  title={Improvements to Target-Based 3D LiDAR to Camera Calibration},
  author={Huang, Jiunn-Kai and Grizzle, Jessy W},
  journal={arXiv preprint arXiv:1910.03126},
  year={2019}
}
```
-->
2. Jiunn-Kai Huang and J. Grizzle, "Improvements to Target-Based 3D LiDAR to Camera Calibration" ([PDF](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/LiDAR2CameraCalibration.pdf))([arXiv](https://arxiv.org/abs/1910.03126))
```
@article{huang2020improvements,
  author={J. {Huang} and J. W. {Grizzle}},
  journal={IEEE Access}, 
  title={Improvements to Target-Based 3D LiDAR to Camera Calibration}, 
  year={2020},
  volume={8},
  number={},
  pages={134101-134110},}
```
