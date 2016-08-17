# Multisensor Extrinsic Calibration Package

A ROS application to estimate the extrinsic parameters (geometric transformations) between a set of sensors, with respect to a global frame using a ball as a calibration target. The calibration process consists of the following stages:

1. each sensor must detect the ball and compute the position of its center;
2. the ball is placed in motion in front of the sensors allowing them to detect its center along sucessive positions, creating at the same time a point cloud of detected centers for each of the sensors;
3. one sensors is chosen as reference and the remainder are calibrated relatively to it.

## Supported Hardware

This package currently supports the following sensors:
- Sick LD-MRS400001
- Sick LMS151
- SwissRanger SR40000
- Point Grey cameras Flea Gigabit Ethernet (tested with the model FL3-GE28S4-C)
- Microsoft Kinect 3D-depth sensor

## ROS Packages

**calibration_gui** is the actual extrinsic calibration package, which requires several other ROS packages:

- **colormap** provides similar functionality as MATLAB colormap;
- **csiro-asl-ros-pkg** provides a ROS driver node for the SICK LD-MRS400001 laser scanner;
- **RCPRG laser drivers** provides a ROS driver node for the SICK LMS151 laser scanner;
- **lidar_segmentation** provides 2D LIDAR segmentation algorithms. Included on [LARtk](http://lars.mec.ua.pt/lartk4/);
- **pointgrey_fl3_ge_28s4_c** provides a ROS driver node for the Point Grey FL3-GE28S4-C camera;
- **swissranger camera-master** provides a ROS driver node for the SwissRanger SR4000 device;

Finally, the **multisensor_fusion** package is not required by the calibration package (**calibration_gui**), but is included since it performs multisensor data combination using the previously estimated geometric transformations.


## Installation

1. Clone (including submodules);
2. Before compiling, install the following system dependencies: [FlyCapture 2.x SDK](https://www.ptgrey.com/support/downloads) and [libmesasr-dev](http://hptg.com/industrial/);
3. Compile the packages

## Video

[![Multisensor Extrinsic Calibration Package Demo](http://i.imgur.com/ZLJIOG6.png)](https://www.youtube.com/watch?v=0umgCqLqCCM)



**ATLAS Project** developed at LAR-UA <br />
http://atlas.web.ua.pt/ <br />
http://lars.mec.ua.pt/ <br />
