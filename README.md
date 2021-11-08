### Overview
This repository holds code of a [ROS](http://www.ros.org) package for point cloud acquisition with  [Gocator3200](http://lmi3d.com/products/gocator/snapshot-sensor) 3D cameras. It is basically a ROS wrapper of the low-level API provided by [LMI Technologies](http://lmi3d.com), the manufacturer of the camera. 

![Camera and cloud at rviz](https://github.com/beta-robots/gocator_3100/blob/master/media/gocator_3100_ros_pkg.png)

### Dependencies
The package has been tested with the following dependencies:
* Ubuntu 20.04
* CMake + gcc
* [ROS](http://wiki.ros.org/indigo/Installation/Ubuntu)
* [Point Cloud Library v1.7](http://www.pointclouds.org/) (shipped with ROS)
* GoSDK library (propietary library from manufacturer LMI Technologies)

To install GoSDK dependency, download the following repo: 

[modified_GO_SDK](https://github.com/Logan-Shi/GO_SDK)

1. first build the sdk: 

```
run ./install.bash
```
2. then set the dir manually at /gocator_3200/CmakeLists.txt:

```
SET(GO_SDK_4 <dir>)
```

### Build
0. ```git clone git@github.com:Logan-Shi/gocator_interface.git```
1. Indicate, by editing the CMakeLists.txt of this package (line 44), where GoSDK library is placed. 
2. The build procedure is the ROS-catkin standard.From your ROS workspace: 
```shell
$ catkin_make
```

### Execute

1. Plug your camera correctly, in terms of power, signal and **grounding!**
2. Properly edit the config/gocator_3200_params.yaml according your configuration and needs. (you can enter 192.168.1.10 for quick param configuration) 
3. set pc local network with a different static IP, e.g. 192.168.1.100.
4. Call the launch file. From the package folder would be
```shell
$ roslaunch launch/gocator_3200.launch
```
