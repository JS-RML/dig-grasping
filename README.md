# Dig-grasping

## 1. Overview
This package presents an autonomous system for singulating and picking thin profile objects, for example, Go stones, capsules and domino blocks, from dense clutter. It is an implementation of **Dig-Grasping**, a new robotic manipulation technique for simultaneously singulating and picking objects from clutter, by leveraging planar quasistatic pushing as a way of direct physical interaction between the object to pick and the robot. As will be provided in this package, dig-grasping exhibits a gripper design with digit asymmetry, realized as a two-fingered gripper with finger length differences. Beyond picking, dig-grasping also presents more complex tasks such as autonomous pick-and-place of Go stones and pick-and-pack of capsules, which are included in this package.

<p align = "center">
<img src="files/Github_Go_stone_pick.gif" width="360" height="202"> 
<img src="files/Github_capsule_pick.gif" width="360" height="202">
<img src="files/domino.gif" width="360" height="202"> 
</p>

<p align = "center">
<img src="files/Github_Go_stone_pick_place.gif" width="360" height="202"> 
<img src="files/Github_capsule_pick_place.gif" width="360" height="202"> 
</p>

**Full Video** can be seen from this [link](https://youtu.be/ea26lKvtYIo). 

## 2. Prerequisites
### 2.1 Hardware
- [**Universal Robot UR10**](https://www.universal-robots.com/products/ur10-robot/)
- [**Robotiq 140mm Adaptive parallel-jaw gripper**](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)
- [**RealSense Camera SR300**](https://github.com/IntelRealSense/realsense-ros)
- [**Customized Finger design**](https://github.com/HKUST-RML/dig-grasping/tree/master/fingertip%20design) features fingertip concavity
- [**Extendable Finger**](https://github.com/HKUST-RML/extendable_finger) for realizing finger length differences during digging
- **Extendable Palm** developed in another package: Shallow-Depth Insertion (Dexterous Ungrasping). See the [link](https://github.com/HKUST-RML/shallow_depth_insertion) for detailed explanation

### 2.2 Software
This implementation requires the following dependencies (tested on Ubuntu 16.04 LTS):
- [**ROS Kinetic**](http://wiki.ros.org/ROS/Installation)
- [**Urx**](https://github.com/SintefManufacturing/python-urx) for UR10 robot control
- [**robotiq_2finger_grippers**](https://github.com/chjohnkim/robotiq_2finger_grippers.git): ROS driver for Robotiq Adaptive Grippers
- [**Mask R-CNN**](https://github.com/matterport/Mask_RCNN) for instance segmentation (also see the dependencies therein). Also download the trained weights for [Go stone](https://hkustconnect-my.sharepoint.com/:u:/g/personal/ztong_connect_ust_hk/Eb7z0WBHf8BOgLfkGKQf1wsBcZgVAwpUTJP7Q9u0y8h5Kw?e=15cEsA) and [capsule](https://hkustconnect-my.sharepoint.com/:u:/g/personal/yhngad_connect_ust_hk/EY5C4hAOm-xNoQ1oHyhArtgBe91wuVaWSf3N2D1fJmcERg?e=aHRnJa).
- [**OpenCV 3.4.1**](https://pypi.org/project/opencv-python/3.4.1.15/) and [**Open3D 0.7.0.0**](http://www.open3d.org/docs/0.7.0/getting_started.html)

**Note**: The online compiler [**Jupyter Notebook**](https://jupyter.org/) is needed to run our program.

## 3. Planar Quasistatic Pushing Simulator
The simulator describes the finger-object interaction where the object is pushed by a position-controlled finger along a straight line. It is implemented in MATLAB. Right now the simulator is demoed with an elliptical or a rectangular object. Given initial pushing conditions - a initial contact position and an orientation of the line of pushing, the simulator will return the trace of the object being pushed, seen from an observer moving together with the finger. For more details, please see the references [\[1\]](https://www.ri.cmu.edu/pub_files/pub2/lynch_kevin_1992_2/lynch_kevin_1992_2.pdf), [\[2\]](https://journals.sagepub.com/doi/pdf/10.1177/0278364918755536).

To run the simulator, enter the folder ```pushing_simulator/elli``` or ```pushing_simulator/rect```,

Run
```
push_elli_demo.m

```
or
```
push_rect_demo.m
```

### **Simulation Parameters**
  - ***a***: length of the object
  - ***b***: width of the object
  - ***N***: gravity of the object
  - ***mu_s***: friction coefficient between the object and the supporting surface
  - ***mu_c***: friction coefficient between the object and the pushing finger
  - ***psi***: orientation of the line of pushing
  - ***d***: initial contact position
  

**An example**

a=0.046, b=0.012, N=0.02; mu_s=0.2, mu_c=0.8, psi=120, d=0.006

<p align = "left">
<img src="files/push_sim_demo.jpg" width="189" height="277"> 
</p>


## 4. A Quick Start of Real Experiments
Create your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):
```
cd ~/catkin_ws/src
git clone https://github.com/HKUST-RML/Dig-grasping.git
cd ..
catkin_make
```
Activate robotiq 2-fingered gripper and RealSense Camera in two different terminals:
```
roslaunch dig-grasping gripper.launch sim:=true
roslaunch realsense2_camera rs_camera.launch align_depth:=true 
```
### Picking
**For picking Go stones**

1. Open another terminal, start Jupyter Notebook via ```jupyter notebook```, and run ```instance_segmentation.ipynb``` for instance segmentation and object pose detection.

2. Start another Jupyter Notebook in a new terminal, and run ```Go_stone_pick.ipynb```.

**For picking capsules**
1. run ```instance_segmentation_capsules.ipynb```
2. run ```Capsule_pick.ipynb```

**For picking domino blocks**
1. run ```Domino_detection.ipynb```
2. run ```Domino_pick.ipynb```

### Placing
Open a terminal, 
```
cd scripts/dexterous_ungrasping/script
```
For Go stone: ```python Go_stone_place.py```

For capsule: ```python Capsule_place.py```

## Maintenance 
For any technical issues, please contact: Zhekai Tong (ztong@connect.ust.hk), and Yu Hin Ng (yhngad@connect.ust.hk).
