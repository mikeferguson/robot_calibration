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
   an LED detector, checkerboard finder, and plane finder. Feature finders
   are plugin-based, so you can create your own.

The second configuration file specifies the configuration for optimization.
This specifies several items:

 * models - Models define how to reproject points. The basic model is a
   kinematic chain. Additional models can reproject through a kinematic
   chain and then a sensor, such as a 3d camera.
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
 * free_frames_initial_values - Defines the initial values for free_frames.
   X, Y, Z offsets are in meters. ROLL, PITCH, YAW are in radians. This is most
   frequently used for setting the initial estimate of the checkerboard position,
   see details below.
 * error_blocks - These define the actual errors to compare during optimization.
   There are several error blocks available at this time:
   * chain3d_to_chain3d - This error block can compute the difference in
     reprojection between two 3D "sensors" which tell us the position of
     certain features of interest. Sensors might be a 3D camera or an arm
     which is holding a checkerboard.
   * chain3d_to_plane - This error block can compute the difference between
     projected 3d points and a desired plane. The most common use case is making
     sure that the ground plane a robot sees is really on the ground.
   * plane_to_plane - This error block is able to compute the difference
     between two planes. For instance, 3d cameras may not have the resolution
     to actually see a checkerboard, but we can align important axis by
     making sure that a wall seen by both cameras is aligned.
   * outrageous - Sometimes, the calibration is ill-defined in certain dimensions,
     and we would like to avoid one of the free parameters from becoming
     absurd. An outrageous error block can be used to limit a particular
     parameter.

### Checkerboard Configuration
When using a checkerboard, we need to estimate the transformation from the
the kinematic chain to the checkerboard. Calibration will be faster and more
accurate if the initial estimate of this transformation is close to the actual
value, especially with regards to rotation.

The simplest way to check your initial estimate is to run the calibration with
only the six DOF of the checkerboard as free parameters. The output values will
be the X, Y, Z, and A, B, C of the transformation. It is important to note that
A, B, C are NOT roll, pitch, yaw -- they are the axis-magnitude representation.
To get roll, pitch and yaw, run the ``to_rpy`` tool with your values of A, B,
and C:
```
rosrun robot_calibration to_rpy A B C
```
This will print the ROLL, PITCH, YAW values to put in for initial values. Then
insert the values in the calibration.yaml:
```
free_frames_initial_values:
 - name: checkerboard
   x: 0.0
   y: 0.225
   z: 0
   roll: 0.0
   pitch: 1.571
   yaw: 0.0
```

## Exported Results

The exported results consist of an updated URDF file, and one or more updated
camera calibration YAML files. By default, these files will by exported into
the /tmp folder, with filenames that include a timestamp of generation. These
files need to be installed in the correct places to be properly loaded.

The [fetch_calibration](https://github.com/fetchrobotics/fetch_ros/tree/indigo-devel/fetch_calibration)
package has an example python script for installing the updated files.

Within the updated URDF file, there are two types of exported results:

 * Changes to free_frames are applied as offsets in the joint origins.
 * Changes to free_params (joint offsets) are applied as "calibration" tags
   in the URDF. In particular, they are applied as "rising" tags. These
   should be read by the robot drivers so that the offsets can be applied
   before joint values are used for controllers. The offsets need to be added
   to the joint position read from the device. The offset then typically
   needs to be subtracted from the commanded position sent to the device.

If your robot does not support the "calibration" tags, it might be possible
to use only free_frames, setting only the rotation in the joint axis to be
free.

## Updating from Indigo

A number of things have been streamlined since Indigo. Some changes in your
capture configuration may be required:

 * GroundPlaneFinder is gone. A simple replacement is to use the PlaneFinder with the
   parameter "min_z" set to -2.0. (all other defaults should work fine)
 * PlaneFinder now supports "debug" parameter, which defaults to false. If you still
   want the point clouds in your bagfile, set this parameter to true.
 * All finders have had their debug topic names updated to include the name of the finder,
   this makes sure that you can run multiple instances and still know where data came from.
   Your RVIZ config may need to be updated.

Your calibration error block config will absolutely need updates:

 * "camera3d_to_arm" is now "chain3d_to_chain3d". In addition to updating the type, the names
   of the sensors have changed: "camera" is now "model_a", and "arm" is "model_b". Order
   of the parameters no longer matters.
 * "camera3d_to_ground" is now "chain3d_to_plane". In addition to updating the type, the
   names of the sensors have changed: "camera" is now "model_a" and there is no sensor
   for ground (the plane parameters are fully accessible instead of being hard-coded). The
   default plane parameters of this error block represent the ground plane.
 * "camera_to_camera" is now "plane_to_plane". In addition to updating the type, the
   names of the sensors have changed from "camera1" and "camera2" to "model_a" and "model_b".

# Status

 * Kinetic Devel Job Status: [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__robot_calibration__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdev__robot_calibration__ubuntu_xenial_amd64/)
 * Kinetic AMD64 Debian Job Status: [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__robot_calibration__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__robot_calibration__ubuntu_xenial_amd64__binary/)
 * Melodic Devel Job Status: [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__robot_calibration__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__robot_calibration__ubuntu_bionic_amd64/)
 * Melodic Coverage: [![codecov](https://codecov.io/gh/mikeferguson/robot_calibration/branch/master/graph/badge.svg)](https://codecov.io/gh/mikeferguson/robot_calibration)
