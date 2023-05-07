# Robot Calibration

This package offers several ROS2 nodes. The primary one is called _calibrate_,
and can be used to calibrate a number of parameters of a robot, such as:

 * 3D Camera intrinsics and extrinsics
 * Joint angle offsets
 * Robot frame offsets

These parameters are then inserted into an updated URDF, or updated camera
configuration YAML in the case of camera intrinsics.

Two additional ROS nodes are used for mobile-base related parameter tuning:

 * _base_calibration_node_ - can determine scaling factors for gyro and track
   width parameters by rotating the robot in place and tracking the actual
   rotation based on the laser scanner view of a wall.
 * _magnetometer_calibration_ - can be used to do hard iron calibration
   of a magnetometer.

## The _calibrate_ node

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

### Configuration

Configuration is typically handled through two sets of YAML files. The first
YAML file specifies the details needed for data capture:

 * chains - The kinematic chains of the robot which should be controlled,
   and how to control them so that we can move the robot to each desired pose
   for sampling.
 * features - The configuration for the various "feature finders" that
   will be making our observations at each sample pose. Current finders include
   an LED detector, checkerboard finder, and plane finder. Feature finders
   are plugin-based, so you can create your own.

The second configuration file specifies the configuration for optimization.
This specifies several items:

 * base_link - Frame used for internal calculations. Typically, the root of the
   URDF is used. Often `base_link`.
 * calibration_steps - In ROS2, multistep calibration is fully supported. The
   parameter "calibration_steps" should be a list of step names. A majority of
   calibrations probably only use a single step, but the step name must still
   be in a YAML list format.

For each calibration step, there are several parameters:

 * models - Models define how to reproject points. The basic model is a
   kinematic chain. Additional models can reproject through a kinematic
   chain and then a sensor, such as a 3d camera. For IK chains, `frame` parameter
   is the tip of the IK chain. The "models" parameter is a list of model names.
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
 * error_blocks - List of error block names, which are then defined under their
   own namespaces.

For each model, the type must be specified. The type should be one of:

 * chain3d - Represents a kinematic chain from the `base_link` to the `frame`
   parameter (which in MoveIt/KDL terms is usually referred to as the `tip`).
 * camera3d - Represents a kinematic chain from the `base_link` to the `frame`
   parameter, and includes the pinhole camera model parameters (cx, cy, fx, fy)
   when doing projection of the points. This model only works if your sensor
   publishes CameraInfo. Further, the calibration obtained when this model is
   used and any of the pinhole parameters are free parameters is only valid if
   the physical sensor actually uses the CameraInfo for 3d projection (this
   is generally true for the Primesense/Astra sensors).

For each error block, the type must be specified. The type should be one of:

 * chain3d_to_chain3d - This error block can compute the difference in
   reprojection between two 3D "sensors" which tell us the position of
   certain features of interest. Sensors might be a 3D camera or an arm
   which is holding a checkerboard. Was previously called "camera3d_to_arm".
 * chain3d_to_mesh - This error block can compute the closeness between
   projected 3d points and a mesh. The mesh must be part of the robot body.
   This is commonly used to align the robot sensor with the base of the robot.
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

#### Checkerboard Configuration
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
ros2 run robot_calibration to_rpy A B C
```
This will print the ROLL, PITCH, YAW values to put in for initial values. Then
insert the values in the calibration.yaml:
```yaml
free_frames_initial_values:
- checkerboard
checkerboard_initial_valus:
  x: 0.0
  y: 0.225
  z: 0
  roll: 0.0
  pitch: 1.571
  yaw: 0.0
```

#### Migrating from ROS1

There are a number of changes in migrating from ROS1 to ROS2. Some of these are
due to differences in the ROS2 system, others are to finally cleanup mistakes
made in earlier version of robot_calibration.

The `chains`, `models`, `free_frames` and `features` parameters used to be lists of YAML
dictionaries. That format is not easily supported in ROS2 and so they are now
lists of string names and the actual dictionaries of information appear under
the associated name. For instance, in ROS1, you might have:

```yaml
models:
 - name: arm
   type: chain
   frame: wrist_roll_link
 - name: camera
   type: camera3d
   frame: head_camera_rgb_optical_frame
```

In ROS2, this becomes:
```yaml
models:
- arm
- camera
arm:
  type: chain3d
  frame: wrist_roll_link
camera:
  type: camera3d
  frame: head_camera_rgb_optical_frame
```

NOTE: the "chain" type has been renamed "chain3d" in ROS2 for consistency (and to allow
a future chain2d).

Multi-step calibration is now fully supported. A new parameter, `calibration_steps` must
be declared as a list of step names. The `models` and free parameters are then specified
for each step. As an example:

```yaml
calibration_steps:
- first_calibration_step
- second_calibration_step
first_calibration_step:
  models: ...
  free_params: ...
second_calibration_step:
  models: ...
  free_params: ...
```

The capture poses can now be specified as YAML. The `convert_ros1_bag_to_yaml` script
can be run in ROS1 to export your ROS1 bagfile as a YAML file that can be loaded in ROS2.

#### Example Configuration

The UBR-1 robot uses this package to calibrate in ROS2. Start with the ``calibrate_launch.py``
in [ubr1_calibration](https://github.com/mikeferguson/ubr_reloaded/tree/ros2/ubr1_calibration)
package.

### Exported Results

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

## The _base_calibration_node_

To run the _base_calibration_node_ node, you need a somewhat open space with a large
(~3 meters wide) wall that you can point the robot at. The robot should be
pointed at the wall and it will then spin around at several different speeds.
On each rotation it will stop and capture the laser data. Afterwards, the
node uses the angle of the wall as measured by the laser scanner to determine
how far the robot has actually rotated versus the measurements from the gyro
and odometry. We then compute scalar corrections for both the gyro and the
odometry.

Node parameters:

 * <code>/base_controller/track_width</code> - this is the default track width.
 * <code>/imu/gyro/scale</code> - this is the initial gyro scale.
 * <code>~min_angle/~max_angle</code> how much of the laser scan to use when
   measuring the wall angle (radians).
 * <code>~accel_limit</code> - acceleration limit for rotation (radians/second^2).

Node topics:

 * <code>/odom</code> - the node subscribes to this odom data. Message type
   is <code>nav_msgs/Odometry</code>.
 * <code>/imu</code> - the node subscribes to this IMU data. Message type
   is <code>sensor_msgs/IMU</code>.
 * <code>/base_scan</code> - the node subscribes to this laser data. Message type
   is <code>sensor_msgs/LaserScan</code>.
 * <code>/cmd_vel</code> - the node publishes rotation commands to this topic, unless
   manual mode is enabled. Message type is <code>geometry_msgs/Twist</code>.

The output of the node is a new scale for the gyro and the odometry. The application
of these values is largely dependent on the drivers being used for the robot. For
robots using _ros_control_ or _robot_control_ there is a track_width parameter
typically supplied as a ROS parameter in your launch file.

## The _magnetometer_calibration_ node

The _magnetometer_calibration_ node records magnetometer data and can compute
the _hard iron_ offsets. After calibration, the magnetometer can be used as
a compass (typically by piping the data through _imu_filter_madgwick_ and
then _robot_localization_).

Node parameters:

 * <code>~rotation_manual</code> - if set to true, the node will not publish command
   velocities and the user will have to manually rotate the magnetometer. Default: false.
 * <code>~rotation_duration</code> - how long to rotate the robot, in seconds.
 * <code>~rotation_velocity</code> - the yaw velocity to rotate the robot, in rad/s.

Node topics:

 * <code>/imu/mag</code> - the node subscribes to this magnetometer data. Message type
   is <code>sensor_msgs/MagneticField</code>.
 * <code>/cmd_vel</code> - the node publishes rotation commands to this topic, unless
   manual mode is enabled. Message type is <code>geometry_msgs/Twist</code>.

The output of the calibration is three parameters, _mag_bias_x_, _mag_bias_y_,
and _mag_bias_z_, which can be used with the <code>imu_filter_madgwick</code> package.

# Status

 * Humble Devel Job Status: [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hdev__robot_calibration__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__robot_calibration__ubuntu_jammy_amd64/)
