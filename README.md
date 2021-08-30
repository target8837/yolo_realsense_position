# yolo_realsense_position
This is a ROS node that takes data from [darknet_ros](https://github.com/leggedrobotics/darknet_ros) and [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper) and fuses it. So that each object found by the YOLO algorithm running on darknet is bundled with the distance to it. 

Realsense-Version
dd

## Limitations and status:
The code in this repository works, but something is wrong in how the parameters are loded from the config file.
