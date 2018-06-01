#yolo_depth_fusion
This is a ROS node that takes data from [darknet_ros](https://github.com/leggedrobotics/darknet_ros) and [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper) and fuses it. So that each object found by the YOLO algorithm running on darknet is bundled with the distance to it. 

Developed during the course CDT406 at Mälardalen University in a project to help blind people navigate using a vision system. [https://github.com/byteofsoren/blind_navi](https://github.com/byteofsoren/blind_navi)

##Limitations and status:
The code in this repository works, but something is wrong in how the parameters are loded from the config file.
