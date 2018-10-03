# Monocular Visual Odometry

----
## Overview
This is a ROS package that implements feature tracking and SFM (Structure From Motion) to build a Monocular Visual Odometry for vehicles. And the trajectory of estimated pose is plotted in rviz. The package has two nodes: One node published images (sensor_msgs::Image) from the KITTI. One subscribes the images and estimates the transformation between each successive frames.

----
## Video Demo

[![Refer_video](https://img.youtube.com/vi/jFF9f5fTl5g/0.jpg)](https://youtu.be/jFF9f5fTl5g)

----
## Pre-request
1. ROS Kinetic
2. rviz
3. Opncv 2.0
4. [jsk rviz plugins](https://jsk-visualization.readthedocs.io/en/latest/jsk_rviz_plugins/index.html) (for plotting tf frame trajectory in rviz)
5. [KITTI Vision Data Benchmark Suite] (http://www.cvlibs.net/datasets/kitti/raw_data.php)

----

## Build and Run
Build :
```
 catkin_make
```


Run:
```
roslaunch visual_odometry visual_odometry.launch

rosrun visual_odometry image_source [image_path]

rosrun visual_odometry visual_odometry [double focal_length] [principel_point_x] [principle_point_y]
```

----
## Pipeline

#### Input
A set of images that are recorded by a camera will be used. The intrinsic parameters (eg. focal length, principle points) of this camera are known. (see [Camera Calibration](https://www.mathworks.com/help/vision/ug/camera-calibration.html))

#### Output
For every pair of the successive frames, the rotation matrix **R** and the translation vector **V** will be calculated to estimate the pose transformation between two frames. The translation vector **V** is associated with its scale factor, which will significantly affect the translation (x, y, z). In order to further solve the scale factor problem, SVO (Fast Semi-Direct Monocular Visual Odometry) are used. For this project, the scale factor is got from the real pose given by KITTI.

### Process
1. Read frames and undistorted them. Then convert the image to gray scale.
2. Use FAST algorithms to detect the features in the frame I(t), and track those features in the frame I(t+1). (See feature detection [FAST](https://docs.opencv.org/2.4/modules/features2d/doc/feature_detection_and_description.html?highlight=fast))
3. Calculates an optical flow for a sparse feature set using the iterative Lucas-Kanade method with pyramids. And calculates an essential matrix from the corresponding points in two images ([OpenCV:: findEssentialMat](https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#Mat%20findEssentialMat(InputArray%20points1,%20InputArray%20points2,%20double%20focal,%20Point2d%20pp,%20int%20method,%20double%20prob,%20double%20threshold,%20OutputArray%20mask))).
4. Recover relative camera rotation and translation from an estimated essential matrix and the corresponding points in two images, using cheirality check. ([OpenCV::recoverPose](https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#Mat%20findEssentialMat(InputArray%20points1,%20InputArray%20points2,%20double%20focal,%20Point2d%20pp,%20int%20method,%20double%20prob,%20double%20threshold,%20OutputArray%20mask)))
5. Take scale information from some external source and update the translation and Rotation.

----

## TO DO
The limitation of the monocular visual odometry is that it can't evaluate the scale, which has a huge impact on translation. In order to resolve the problem, SVO (Fast Semi-Direct Monocular Visual Odometry) can be used. And the data from the IMU will provide a relative reliable scale value. The other problem is that: visual odometry alone is more practical in the static environment. If other objects (features) are moving, then the estimated pose will messed up. Thus, more sensors (eg. IMU) are needed for sensor fusion to optimize the pose result.

----
## Thanks
* [Avi Singh's blog](https://avisingh599.github.io/vision/monocular-vo/): Monocular Visual Odometry using OpenCV
