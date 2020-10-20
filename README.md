# Dig-grasping

## 1. Overview
This package presents an autonomous system for singulating and picking thin profile objects, for example, Go stones and capsules, from dense clutter. It is an implementation of **Dig-Grasping**, a new robotic manipulation technique for simultaneously singulating and picking objects from clutter, by leveraging planar quasistatic pushing as a way of direct physical interaction between the object to pick and the robot. As will be provided in this package, dig-grasping exhibits a gripper design with digit asymmetry, realized as a two-fingered gripper with finger length differences. Beyond picking, dig-grasping also presents more complex tasks such as autonomous pick-and-place of Go stones and pick-and-pack of capsules, which are included in this package.

**Full Video** can be seen from this [link](https://youtu.be/dwZhAJ4iFk4). 

## 2. Prerequisites
### 2.1 Hardware
- [**Universal Robot UR10**](https://www.universal-robots.com/products/ur10-robot/)
- [**Robotiq 140mm Adaptive parallel-jaw gripper**](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)
- [**RealSense Camera SR300**](https://github.com/IntelRealSense/realsense-ros)
- [**Extendable Finger**] for realizing finger length differences during digging
- **Extendable Palm** developed in another package: Shallow-Depth Insertion (Dexterous Ungrasping). See the [link](https://github.com/HKUST-RML/shallow_depth_insertion) for detailed explanation

### 2.2 Software
This implementation requires the following dependencies (tested on Ubuntu 16.04 LTS):
- [**ROS Kinetic**](http://wiki.ros.org/ROS/Installation)
- [**Urx**](https://github.com/SintefManufacturing/python-urx) for UR10 robot control
- [**robotiq_2finger_grippers**](https://github.com/chjohnkim/robotiq_2finger_grippers.git): ROS driver for Robotiq Adaptive Grippers
- [**Mask R-CNN**](https://github.com/matterport/Mask_RCNN) for instance segmentation (also see the dependencies therein). Also download the trained weights for [Go stone](https://hkustconnect-my.sharepoint.com/:u:/g/personal/ztong_connect_ust_hk/Eb7z0WBHf8BOgLfkGKQf1wsBcZgVAwpUTJP7Q9u0y8h5Kw?e=15cEsA) and [capsule].
- [**OpenCV 3.4.1**](https://pypi.org/project/opencv-python/3.4.1.15/) and [**Open3D 0.7.0.0**](http://www.open3d.org/docs/0.7.0/getting_started.html)

**Note**: The online compiler [**Jupyter Notebook**](https://jupyter.org/) is needed to run our program.

## 3. A Quick Start
Create your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):
```
cd ~/catkin_ws/src
git clone https://github.com/HKUST-RML/Dig-grasping.git
cd ..
catkin_make
```
Activate robotiq 2-fingered gripper:
```
roslaunch dig-grasping gripper.launch sim:=true
```

## Maintenance 
For any technical issues, please contact: Zhekai Tong (ztong@connect.ust.hk), and Yu Hin Ng (yhngad@connect.ust.hk).
