# Robot Calibration

This package offers calibration of a number of parameters of a robot, such as:

 * 3D Camera intrinsics and extrinsics
 * Joint angle offsets
 * Robot frame offsets

These parameters are then inserted into an updated URDF, or updated camera
configuration YAML in the case of camera intrinsics.

## Overview

Calibration works in two steps. The first step involves the capture of data
samples from the robot. Each "sample" comprises the measured joint positions
of the robot and two or more "observations". An observation is a collection
of points that have been detected by a "sensor". For instance, a robot could
use a camera and an arm to "detect" the pose of corners on a checkerboard.
In the case of the camera sensor, the collection of points is simply the
detected positions of each corner of the checkerboard, relative to the pose
of the camera reference frame. For the arm, it is assumed that the checkerboard
is fixed relative to a virtual frame which is fixed relative to the end
effector of the arm. Within the virtual frame, we know the position of each
point of the checkerboard corners.

The second step of calibration involves optimization of the robot parameters
to minimize the errors. Errors are defined as the difference in the pose
of the points based on reprojection throuhg each sensor. In the case of our
checkerboard above, the transform between the virtual frame and the end
effector becomes additional free parameters. By estimating these parameters
alongside the robot parameters, we can find a set of parameters such that
the reprojection of the checkerboard corners through the arm is as closely
aligned with the reprojection through the camera (and any associated
kinematic chain, for instance, a pan/tilt head).

## Configuration

Configuration is typically handled through two sets of YAML files. The first
YAML file specifies the details needed for data capture:

 * chains - The kinematic chains of the robot which should be controlled,
   and how to control them so that we can move the robot to each desired pose
   for sampling.
 * feature_finders - The configuration for the various "feature finders" that
   will be making our observations at each sample pose. Current finders include
   an LED detector and a checkerboard finder.

The second configuration file specifies the configuration for optimization.
This specifies several items:

 * models - Models define how to reproject points. The basic model is a
   kinematic chain. Additional models can reproject through a kinematic
   chain and then a sensor, such as a camera.
 * free_params - Defines the names of single-value free parameters. These
   can be the names of a joint for which the joint offset should be calculated,
   camera parameters such as focal lengths, or other parameters, such as
   driver offsets for Primesense devices.
 * free_frames - Defines the names of multi-valued free parameters that
   are 6-d transforms. Also defines which axis are free. X, Y, and Z can all
   be independently set to free parameters. Roll, pitch and yaw can also be
   set free, however it is important to note that because calibration
   internally uses an angle-axis representation, either all 3 should be set
   free, or only one should be free. You should never set two out of three
   to be free parameters.
 * error_blocks - These define the actual errors to compare during optimization.
   There are several error blocks available at this time:
   * camera3d_to_arm - This error block can compute the difference in
     reprojection between a 3D camera and a kinematic chain which is holding
     the projected points.
   * outrageous - Sometimes, the calibration is ill-defined in certain dimensions,
     and we would like to avoid one of the free parameters from becoming
     absurd. An outrageous error block can be used to limit a particular
     parameter.

## Exported Results

The exported results consist of an updated URDF file, and one or more updated
camera calibration YAML files. By default, these files will by exported into
the /tmp folder, with filenames that include a timestamp of generation. These
files need to be installed in the correct places to be properly loaded.

The [fetch_calibration](https://github.com/fetchrobotics/fetch_ros/tree/indigo-devel/fetch_calibration)
package has an example python script for installing the updated files.

# Status

 * Devel Job Status: [![Build Status](http://build.ros.org/buildStatus/icon?job=Idev__robot_calibration__ubuntu_trusty_amd64)](http://build.ros.org/job/Idev__robot_calibration__ubuntu_trusty_amd64/)
 * AMD64 Debian Job Status: [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__robot_calibration__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__robot_calibration__ubuntu_trusty_amd64__binary/)
