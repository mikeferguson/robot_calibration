# Robot Calibration

This package offers several ROS2 nodes. The primary one is called _calibrate_,
and can be used to calibrate a number of parameters of a robot, such as:

 * 3D Camera intrinsics and extrinsics
 * Joint angle offsets
 * Robot frame offsets

These parameters are then inserted into an updated URDF, or updated camera
configuration YAML in the case of camera intrinsics.

Two additional ROS nodes are used for mobile-base related parameter tuning:

 * _base_calibration_node_ - can determine scaling factors for wheel diameter,
   track width and gyro gain by moving and rotating the robot while tracking
   the actual movement based on the laser scanner view of a wall.
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
is fixed relative to a virtual ``checkerboard`` frame which is fixed relative
to the end effector of the arm. Within the virtual frame, we know the ideal
position of each point of the checkerboard corners since the checkerboard
is of known size.

The second step of calibration involves optimization of the robot parameters
to minimize the errors. Errors are defined as the difference in the pose
of the points based on reprojection throuhg each sensor. In the case of our
checkerboard above, the transform between the virtual frame and the end
effector becomes additional free parameters. By estimating these parameters
alongside the robot parameters, we can find a set of parameters such that
the reprojection of the checkerboard corners through the arm is as closely
aligned with the reprojection through the camera (and any associated
kinematic chain, for instance, a pan/tilt head).

Configuration is typically handled through two sets of YAML files: usually
called ``capture.yaml`` and ``calibrate.yaml``.

If you want to manually move the robot to poses and capture each time you
hit ENTER on the keyboard, you can run robot calibration with:

```
ros2 run robot_calibration calibrate --manual --ros-args --params-file path-to-capture.yaml --params-file path-to-calibrate.yaml
```

More commonly, you will generate a third YAML file with the capture pose
configuration (as documented below in the section "Calibration Poses"):

```
ros2 run robot_calibration calibrate path-to-calibration-poses.yaml --ros-args --params-file path-to-capture.yaml --params-file path-to-calibrate.yaml
```

This is often wrapped into a ROS 2 launch file, which often records
a bagfile of the observations allowing to re-run just the calibration part
instead of needing to run capture each time. For an example, see the
UBR-1 example in the next section.

### Example Configuration

All of the parameters that can be defined in the capture and calibrate steps
are documented below, but sometimes it is just nice to have a full example.
The UBR-1 robot uses this package to calibrate in ROS2. Start with
the ``calibrate_launch.py`` in
[ubr1_calibration](https://github.com/mikeferguson/ubr_reloaded/tree/ros2/ubr1_calibration).

### Capture Configuration

The ``capture.yaml`` file specifies the details needed for data capture:

 * ``chains`` - A parameter listing the names of the kinematic chains of the
   robot which should be controlled.
 * ``features`` - A parameter listing the names of the various "feature finders"
   that will be making our observations at each sample pose.

Each of these chains and features is then defined by a parameter block of the
same name, for example:

```yaml
robot_calibration:
  ros_parameters:
    # List of chains
    chains:
    - arm
    # List of features
    features:
    - checkerboard_finder
    # Parameter block to define the arm chain
    arm:
      topic: /arm_controller/follow_joint_trajectory
      joints:
      - first_joint
      - second_joint
    # Parameter block to define the feature finder:
    checkerboard_finder:
      type: robot_calibration::CheckerboardFinder
      topic: /head_camera/depth_registered/points
      camera_sensor_name: camera
      chain_sensor_name: arm
```

#### Chain Parameters

For each chain, the following parameters can be defined:

 * ``topic`` - The namespace of the ``control_msgs::FollowJointTrajectory``
   server used to control this chain.
 * ``planning_group`` - Optional parameter, when set to a non-empty string
   ``robot_calibration`` will call MoveIt to plan a collision free path from
   the current robot pose to the next capture pose. When this parameter is
   not set, the trajectory simply interpolates from the current pose to the
   next capture pose without collision awareness - so you need to be careful
   when defining your series of capture poses.
 * ``joints`` - A list of joints that this group comprises.

#### Finder Parameters

At a minimum, the following parameters must be set for all finders:

 * ``type`` - Name of the plugin to load.
 * ``camera_sensor_name`` - Every finder outputs observations from some
   sensor - this name must match the name used later in ``calibrate.yaml``.
 * ``chain_sensor_name`` - Every finder outputs observations from some
   chain - this name must match the name used later in ``calibrate.yaml``.
 * ``debug`` - Most finders have a debug parameter which will insert the
   raw image or point cloud into the observation. This makes the capture
   bagfile larger but aids in debugging.

The following types are currently included with ``robot_calibration``
although you can create your own plugins. Each finder has it's own
additional parameters:

 * ``robot_calibration::CheckerboardFinder`` - Detects checkerboards in
   a point cloud.
    * ``topic`` - Name of topic of type ``sensor_msgs::PointCloud2``.
    * ``points_x`` - Number of corners in the X direction of the checkerboard.
    * ``points_y`` - Number of corners in the Y direction of the checkerboard.
    * ``size`` - Size of checkerboard squares, in meters.
 * ``robot_calibration::CheckerboardFinder2d`` - Detects checkerboards in
   an image:
    * ``topic`` - Name of topic of type ``sensor_msgs::Image``.
    * ``points_x`` - Number of corners in the X direction of the checkerboard.
    * ``points_y`` - Number of corners in the Y direction of the checkerboard.
    * ``size`` - Size of checkerboard squares, in meters.
 * ``robot_calibration::LedFinder`` - controls and detects a series of LEDs,
   which can be a built-in alternative to having a robot hold the checkerboard.
    * ``gripper_led_action`` - Namespace of the gripper LED action server.
    * ``topic`` - Name of topic of type ``sensor_msgs::PointCloud2``.
    * ``max_error`` - Maximum distance detected LED can be from expected pose,
      in meters.
    * ``max_inconsistency`` - Maximum relative difference between two LEDs in
      the same capture pose, in meters.
    * ``max_iterations`` - Maximum number of times to toggle the LEDs for a given
      capture pose.
    * ``gripper_led_frame`` - The robot link which the ``leds`` are defined in.
    * ``leds`` - Definition of the LED poses. For each LED, you need to specify
      a ``code`` which is sent to the action server to turn that LED on, as well
      as ``x``, ``y``, and ``z`` offsets relative to ``gripper_led_frame`` for
      the expected pose of that LED. These values will be used to generate
      the chain observation.
 * ``robot_calibration::PlaneFinder`` - Detects planes in a point cloud. This
   will filter out points outside the limits, and then iteratively find the
   largest plane until a desired one is found. This is commonly used to align
   a sensor with the ground.
    * ``topic`` - Name of topic of type ``sensor_msgs::PointCloud2``.
    * ``points_max`` - Maximum number of points to use in the observation
    * The cloud can be pre-filtered using the ``min_x``, ``max_x``, ``min_y``,
      ``max_y``, ``min_z``, and ``max_z`` parameters.
    * The desired orientation of the plane can be used by setting ``normal_a``,
      ``normal_b``, ``normal_c`` parameters - if all are 0, the biggest plane
      will be selected regardless of orientation. This is particularly useful
      if the robot might be looking partially at the wall as well as the desired
      floor surface.
    * ``normal_angle`` - If a desired orientation vector is set, the candidate
      plane normal must be within this angle of the desired normal, in radians.
 * ``robot_calibration::ScanFinder`` - Detects points in a laser scan, and then
   repeats them vertically. This can be used to align a laser scanner against
   a plane detected by a 3d camera.
    * ``topic`` - Name of topic of type ``sensor_msgs::LaserScan``.
    * ``transform_frame`` -- Frame to transform the laser scan into, usually
      ``base_link``.
    * ``min_x``, ``max_x``, ``min_y``, and ``max_y`` are used to limit the
      laser scan points that are used. They are defined in the ``transform_frame``.
    * ``z_repeats`` - How many times to copy the points vertically.
    * ``z_offset`` - Distance between repeated points.

Additionally, any finder that subscribes to a depth camera has the following parameters:

 * ``camera_info_topic``: The topic name for the camera info.
 * ``camera_driver``: Namespace of the camera driver, only used for Primesense-like
   devices which have ``z_offset_mm`` and ``z_scaling`` parameters.

### Calibration Configuration

The ``calibrate.yaml`` configuration file specifies the configuration for
optimization. This specifies several items:

 * ``base_link`` - Frame used for internal calculations. Typically, the root of the
   URDF is used. Often `base_link`.
 * ``calibration_steps`` - In ROS 2, multistep calibration is fully supported. The
   parameter ``calibration_steps`` should be a list of step names. A majority of
   calibrations probably only use a single step, but the step name must still
   be in a YAML list format.

```yaml
robot_calibration:
  ros__parameters:
    base_link: torso_lift_link
    calibration_steps:
    - single_calibration_step
    single_calibration_step:
      models:
      - first_model
      first_model:
        type: first_model_type
```

For each calibration step, there are several parameters:

 * ``models`` - List of model names. Each model will then be defined in a
   parameter block defined by the name. Models define how to reproject
   observation points into the fixed frame. The basic model is a
   kinematic chain. Additional models can reproject through a kinematic
   chain and then a sensor, such as a 3d camera. Once loaded, models
   will be used by the error blocks to compute the reprojection errors
   between different sensor observations.
 * ``free_params`` - Defines the names of single-value free parameters. These
   can be the names of a joint for which the joint offset should be calculated,
   camera parameters such as focal lengths or the driver offsets for
   Primesense devices. If attempting to calibrate the length of a robot
   link, use `free_frames` to define the axis that is being calibrated.
 * ``free_frames`` - Defines the names of multi-valued free parameters that
   are 6-d transforms. Also defines which axis are free. X, Y, and Z can all
   be independently set to free parameters. Roll, pitch and yaw can also be
   set free, however it is important to note that because calibration
   internally uses an angle-axis representation, either all 3 should be set
   free, or only one should be free. You should never set two out of three
   to be free parameters.
 * ``free_frames_initial_values`` - Defines the initial offset values for
   ``free_frames``. X, Y, Z offsets are in meters. ROLL, PITCH, YAW are in
   radians. This is most frequently used for setting the initial estimate
   of the checkerboard position, see details below.
 * ``error_blocks`` - List of error block names, which are then defined
   under their own namespaces.

For each model, the `type` must be specified. The type should be one of:

 * ``chain3d`` - Represents a kinematic chain from the `base_link` to the `frame`
   parameter (which in MoveIt/KDL terms is usually referred to as the `tip`).
 * ``camera3d`` - Represents a kinematic chain from the `base_link` to the `frame`
   parameter, and includes the pinhole camera model parameters (cx, cy, fx, fy)
   when doing projection of the points. This model only works if your sensor
   publishes CameraInfo. Further, the calibration obtained when this model is
   used and any of the pinhole parameters are free parameters is only valid if
   the physical sensor actually uses the CameraInfo for 3d projection (this
   is generally true for the Primesense/Astra sensors).
 * ``camera2d`` - Similar to `camera3d`, but for a 2d finder. Currently only
   works with the output of the ``CheckerboardFinder2d``.

For each error block, the ``type`` must be specified. In addition to the
``type`` parameter, each block will have additional parameters:

 * ``chain3d_to_chain3d`` - The most commonly used error block type.
   This error block can compute the difference in reprojection
   between two 3D "sensors" which tell us the position of
   certain features of interest. Sensors might be a 3D camera or an arm
   which is holding a checkerboard. Was previously called "camera3d_to_arm":
    * `model_a` - First `chain3d` or `camera3d` model to use in computing
      reprojection error.
    * `model_b` - Second `chain3d` or `camera3d` model to use in computing
      reprojection error.
 * ``chain3d_to_camera2d``- Currently only used for the `CheckerboardFinder2d`:
    * `model_2d` - `camera2d` model to use in computing reprojection error.
    * `model_3d` - `chain3d` or `camera3d` model to use in computing
      reprojection error.
    * `scale` - Scalar to multiply summed error by - note that error computed
      in this block is in *pixel* space, rather than *metric* space like most
      other error blocks.
 * ``chain3d_to_mesh`` - This error block type can compute the closeness between
   projected 3d points and a mesh. The mesh must be part of the robot body.
   This is commonly used to align the robot sensor with the base of the robot,
   using points that were found by the `RobotFinder` plugin:
    * `model` - `chain3d` or `camera3d` model to use in computing reprojection
      error.
    * `link_name` -Name of the link in the URDF for which mesh to use.
 * ``chain3d_to_plane`` - This error block can be used to compare projected
   points to a plane. Each observation point is reprojected, then the sum
   of distance to plane for each point is computed. The most common use case
   is making sure that the ground plane a robot sees is really on the ground:
    * `model` - The `camera3d` model for reprojection.
    * `a`, `b`, `c`, `d` - Parameters for the desired plane equation, in the
      form `ax + by + cz + d = 0`.
    * `scale` - Since the error computed is a distance from the plane over
      many points, scaling the error relative to other error blocks is often
      required.
 * ``plane_to_plane`` - This error block is able to compute the difference
   between two planes. For instance, 3d cameras may not have the resolution
   to actually see a checkerboard, but we can align important axis by
   making sure that a wall seen by both cameras is aligned. For each observation,
   the points are assumed to form a plane:
    * `model_a` - First `chain3d` or `camera3d` model to use in computing
      reprojection error.
    * `model_b` - Second `chain3d` or `camera3d` model to use in computing
      reprojection error.
    * `normal_scale` - The normal error is computed as the difference between
      the two plane normals and then multiplied by this scalar.
    * `offset_scale` - The offset error is computed as the distance from
      the centroid of the first plane to the second plane and then
      multiplied by this scalar.
 * ``outrageous`` - Sometimes, the calibration is ill-defined in certain dimensions,
   and we would like to avoid one of the free parameters from becoming
   absurd. An outrageous error block can be used to limit a particular
   parameter:
    * `param` - Free parameter to monitor.
    * `joint_scale` - If `param` is a joint name, multiply the free param value
      by this scalar.
    * `position_scale` - If `param` is a free frame, multiply the metric distance
      in X, Y, Z by this scalar.
    * `rotation_scale` - If `param` is a free frame, multiply the angular distance
      of the free parameter value by this scalar.

### Calibration Poses

The final piece of configuration is the actual poses from which the robot should
capture data. This YAML file can be created by running the `capture_poses` script.
You will be prompted to move the robot to the desired pose and press ENTER, when
done collecting all of your poses, you can type EXIT.
This will create `calibration_poses.yaml` which is an array of capture poses:

```yaml
- features: []
  joints:
  - first_joint
  - second_joint
  positions:
  - -0.09211555123329163
  - 0.013307283632457256
- features: []
  joints:
  - first_joint
  - second_joint
  positions:
  - -1.747204065322876
  - -0.07186950743198395
```

By default, every finder is used for every capture pose. In some cases, you might
want to specify specific finders by editing the `features`:

```yaml
# This sample pose uses only the `ground_plane_finder` feature finder
- features:
  - ground_plane_finder
  joints:
  - first_joint
  - second_joint
  positions:
  - -0.09211555123329163
  - 0.013307283632457256
# This sample pose will use all features
- features: []
  joints:
  - first_joint
  - second_joint
  positions:
  - -1.747204065322876
  - -0.07186950743198395
```

#### Checkerboard Configuration

When using a checkerboard, we need to estimate the transformation from the
the tip of the kinematic chain to the virtual ``checkerboard`` frame.
Calibration will be faster and more accurate if the initial estimate of
this transformation is close to the actual
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
checkerboard_initial_values:
  x: 0.0
  y: 0.225
  z: 0
  roll: 0.0
  pitch: 1.571
  yaw: 0.0
```

[This tool](https://markhedleyjones.com/projects/calibration-checkerboard-collection)
can be helfpul for creating checkerboards.

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
(~3 meters wide) wall that you can point the robot at.

Starting with the `0.10` release of `robot_calibration`, the actual movements the
robot does can be programmed via the `calibration_steps` parameter:

 * <code>calibration_steps</code> - should be a list of string names of calibration
   steps to run.
 * <code>step_name/type</code> - should be either `spin` or `rollout`.
 * <code>step_name/velocity</code> - velocity to move the robot. This will be
   interpreted as angular velocity for `spin` steps and linear velocity for `rollout`
   steps.
 * <code>step_name/rotations</code> - only valid for `spin` steps. Number of
   rotations to complete at given `velocity`.
 * <code>step_name/distance</code> - only valid for `rollout` steps. Distance
   in meters, to rollout the robot.

Additional parameters:

 * <code>accel_limit</code> - acceleration limit for `spin` steps (radians/second^2).
 * <code>linear_accel_limit</code> - acceleration limit for `rollout` steps (meters/second^2).
 * <code>min_angle/max_angle</code> how much of the laser scan to use when
   measuring the wall angle (radians).

Node topics:

 * <code>/odom</code> - the node subscribes to this odom data. Message type
   is <code>nav_msgs/Odometry</code>.
 * <code>/imu</code> - the node subscribes to this IMU data. Message type
   is <code>sensor_msgs/IMU</code>.
 * <code>/base_scan</code> - the node subscribes to this laser data. Message type
   is <code>sensor_msgs/LaserScan</code>.
 * <code>/cmd_vel</code> - the node publishes rotation commands to this topic, unless
   manual mode is enabled. Message type is <code>geometry_msgs/Twist</code>.

The output of the node is a series of scalars to apply (where a value of 1.0 means
there is currently no error):

```
[base_calibration_node-1] track_width_scale: 0.986743
[base_calibration_node-1] imu_scale: 0.984465
[base_calibration_node-1] rollout_scale: 0.981911
```

The application of these values is largely dependent on the drivers being used
for the robot. For robots using _ros_control_ or _robot_control_ there is a
`track_width` parameter typically supplied as a ROS parameter in your launch file.

Note: in ROS 1, these scalars were pre-multiplied by the existing `track_width`
or `imu_scale` - however, with the lack of a unified parameter server in ROS 2,
this is no longer done.

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

### Migrating from ROS1

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
