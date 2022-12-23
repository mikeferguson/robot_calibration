^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_calibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* calibrate falling attributes as well if they exist (`#144 <https://github.com/mikeferguson/robot_calibration/issues/144>`_)
  For continuous joints actually both flags could be specified.
* Changes for Ubuntu 22.04 (`#141 <https://github.com/mikeferguson/robot_calibration/issues/141>`_)
  * no longer use c++11
  * Add missing boost header
  * Switch to non deprecated pluginlib header
* Contributors: Jochen Sprickerhof, Michael Görner

0.7.1 (2022-06-15)
------------------
* enhance alignment (`#130 <https://github.com/mikeferguson/robot_calibration/issues/130>`_)
  * handle NANs in laser data
  * add r2_tolerance
  * add spinOnce so that we can run single threaded
  * exit loop if shutting down
* fix regression in manual calibration (`#129 <https://github.com/mikeferguson/robot_calibration/issues/129>`_)
* fix warnings reported on ros.org buildfarm (`#128 <https://github.com/mikeferguson/robot_calibration/issues/128>`_)
* improve base calibration alignment (`#127 <https://github.com/mikeferguson/robot_calibration/issues/127>`_)
  * add feature to align to arbitrary angle
  * parameterize things, including verbose
* refactor base_calibration into separate class (`#126 <https://github.com/mikeferguson/robot_calibration/issues/126>`_)
* major refactor into re-usable components (`#125 <https://github.com/mikeferguson/robot_calibration/issues/125>`_)
  * add getPosesFromBag function
  * add captureManager to clean up code and improve re-usability
  * split out exportResults as a function
  * for all models, renamed name() to getName(), added getType() function
  * add getCameraNames() function to optimizer, so that we can:
  * output camera calibrations for ALL cameras
* Contributors: Michael Ferguson

0.7.0 (2022-04-20)
------------------
* filter planes by closeness to desired normal (`#124 <https://github.com/mikeferguson/robot_calibration/issues/124>`_)
* add finder for laser scan data (`#123 <https://github.com/mikeferguson/robot_calibration/issues/123>`_)
  best way to use this right now is to create point laser scanner at a
  wall, create a plane from the laser scan, and align it with a plane
  that has been seen by a camera
* sample observation more evenly (`#122 <https://github.com/mikeferguson/robot_calibration/issues/122>`_)
  the plane finder previously took points that were equally distributed by
  point cloud index - however this doesn't actually always represent a
  point cloud that is geometry well distributed. change the sampling so
  that we make one or more passes through the cloud sampling points that
  are at least a minimum distance away from the already sampled points
* refactor plane finder (`#121 <https://github.com/mikeferguson/robot_calibration/issues/121>`_)
  * use new eigen_geometry.h functions
  * use ransac to find the best plane
* refactor and improve plane_to_plane cost function (`#120 <https://github.com/mikeferguson/robot_calibration/issues/120>`_)
  * use Eigen rather than OpenCV
  * break plane parameter functions into separate header for reuse
  * make final residual distance between planes
  * make plane parameters always in same direction
* make it easier to exit out of calibration (`#119 <https://github.com/mikeferguson/robot_calibration/issues/119>`_)
* parameterize max_num_iterations (`#118 <https://github.com/mikeferguson/robot_calibration/issues/118>`_)
* add chain3dToMesh calibration (`#116 <https://github.com/mikeferguson/robot_calibration/issues/116>`_)
  * add cost function
  * add mesh loader utilities
  * add node to visualize meshes load
* allow using the same camera model with different finders (`#112 <https://github.com/mikeferguson/robot_calibration/issues/112>`_)
  the use case here is that a single camera is used in multiple
  finders. When that happens we want the same parameters to
  be used when calibrating the camera intrinsics, but we
  need a different camera name to differentiate which
  observation should be associated with which error block
  (this is especially true for the upcoming robot/plane
  finder)
* add robot finder (`#111 <https://github.com/mikeferguson/robot_calibration/issues/111>`_)
  The robot finder can be used to find the base of the robot and the floor at the same time
* increase marker size (`#117 <https://github.com/mikeferguson/robot_calibration/issues/117>`_)
* minor improvements and fixes for viz node (`#110 <https://github.com/mikeferguson/robot_calibration/issues/110>`_)
  * move publishers earlier and make latching
  * don't crash if no points in observation after projecting
* refactor and improve plane finder (`#109 <https://github.com/mikeferguson/robot_calibration/issues/109>`_)
  * break up the find() function so that it is more re-usable
  * change points_max parameter to an integer rather than double
  * actually find the plane rather than just assuming everything is part of the plane
* bump cmake version to silence warnings on noetic (`#107 <https://github.com/mikeferguson/robot_calibration/issues/107>`_)
* Updated the press key message (`#106 <https://github.com/mikeferguson/robot_calibration/issues/106>`_)
* Add exception when the chain cannot be created (`#105 <https://github.com/mikeferguson/robot_calibration/issues/105>`_)
* Contributors: Gerardo Puga, Michael Ferguson

0.6.5 (2021-10-30)
------------------
* add support for static camera calibration (`#101 <https://github.com/mikeferguson/robot_calibration/issues/101>`_)
* remove references to chain3d_to_arm (`#99 <https://github.com/mikeferguson/robot_calibration/issues/99>`_)
* Contributors: Michael Ferguson

0.6.4 (2020-11-02)
------------------
* improve visualization (`#91 <https://github.com/mikeferguson/robot_calibration/issues/91>`_)
  * publish joint states for viz
  * publish point clouds
* use all features when features are unspecified (`#92 <https://github.com/mikeferguson/robot_calibration/issues/92>`_)
* only accept organized clouds, fixes `#79 <https://github.com/mikeferguson/robot_calibration/issues/79>`_ (`#90 <https://github.com/mikeferguson/robot_calibration/issues/90>`_)
* catch by reference to silence warnings (`#89 <https://github.com/mikeferguson/robot_calibration/issues/89>`_)
* fix opencv build issue (`#88 <https://github.com/mikeferguson/robot_calibration/issues/88>`_)
* update package.xml for noetic (`#87 <https://github.com/mikeferguson/robot_calibration/issues/87>`_)
  orocos-kdl is now a system dependency,
  rosdep key has changed
* Contributors: Michael Ferguson

0.6.3 (2020-04-27)
------------------
* kinetic requires C++11, but doesn't specify it (`#85 <https://github.com/mikeferguson/robot_calibration/issues/85>`_)
* note topics being published/subscribed
* some fixes for magnetometer cal (`#84 <https://github.com/mikeferguson/robot_calibration/issues/84>`_)
  * the spinOnce was needed
  * exit properly on CTRL-C
* add magnetometer calibration node (`#83 <https://github.com/mikeferguson/robot_calibration/issues/83>`_)
* remove readme, top level one has docs
* add travis and code coverage (`#80 <https://github.com/mikeferguson/robot_calibration/issues/80>`_)
* export feature_finders lib
* Contributors: Michael Ferguson

0.6.2 (2020-01-14)
------------------
* Merge pull request `#75 <https://github.com/mikeferguson/robot_calibration/issues/75>`_ from mikeferguson/mute_warnings
  fix warnings about build type
* fix warnings about build type
* Merge pull request `#74 <https://github.com/mikeferguson/robot_calibration/issues/74>`_ from mikeferguson/fix_tests
  fix tests broken by `#71 <https://github.com/mikeferguson/robot_calibration/issues/71>`_
* fix tests broken by `#71 <https://github.com/mikeferguson/robot_calibration/issues/71>`_
* Merge pull request `#71 <https://github.com/mikeferguson/robot_calibration/issues/71>`_ from Naoki-Hiraoka/fix-calculation-of-frame_offset
  Fix calculation of frame offset
* Merge pull request `#73 <https://github.com/mikeferguson/robot_calibration/issues/73>`_ from mikeferguson/multi-step
  Support multi-step optimization
* Merge pull request `#68 <https://github.com/mikeferguson/robot_calibration/issues/68>`_ from d-walsh/bugfix/isnan_error
  Fixed isnan() error on Kinetic
* refactor mutli-step support
* enable multi-step optimization
* fix frame calculation in getChainFK()
* fix calculation of frame_offset
* Fixed isnan() error on Kinetic
* Contributors: David Walsh, Michael Ferguson, Naoki-Hiraoka

0.6.1 (2019-11-19)
------------------
* Merge pull request `#70 <https://github.com/mikeferguson/robot_calibration/issues/70>`_ from Naoki-Hiraoka/enable-to-change-driver-name
  Enable to change driver name
* Merge pull request `#69 <https://github.com/mikeferguson/robot_calibration/issues/69>`_ from Naoki-Hiraoka/allow-multiple-checkerboards
  Use multiple checkerboards
* enable to change driver name
* allow multiple checkerboards
* Merge pull request `#56 <https://github.com/mikeferguson/robot_calibration/issues/56>`_ from mikeferguson/coverage
  add code coverage testing
* update code_coverage to be test_depend
* add code coverage testing
* Contributors: Michael Ferguson, Naoki-Hiraoka

0.6.0 (2018-07-09)
------------------
* install our new tools
* add depend on visualization_msgs
* Merge pull request `#63 <https://github.com/mikeferguson/robot_calibration/issues/63>`_ from mikeferguson/checkerboards_that_work
  Make checkerboards actually generic
* Merge pull request `#62 <https://github.com/mikeferguson/robot_calibration/issues/62>`_ from mikeferguson/chain_manager_state_fix
  make sure we get valid joint_states
* make sure we get valid joint_states
  * invalid old state, wait for new message
  * fixes `#61 <https://github.com/mikeferguson/robot_calibration/issues/61>`_
* Merge pull request `#59 <https://github.com/mikeferguson/robot_calibration/issues/59>`_ from saurabhbansal90/master
  Update led_finder.cpp
* Update led_finder.cpp
* additional tests on camera_info
* fix build in kinetic
* remove entirely unused data functions header
* fix corrupted license file
* additional warning not previously flagged
* buildfarm is really picky, fix another signed comparison
* fix signed comparison warning in tests
* add tool to visualize bagfile
* break out load_bag function for reuse
* towards working checkerboards
  * unhack the checkerboard finder, so that points are in x/y only
  * add free_frames_initial_values parameter for setting initial
  offset of checkerboard frame
* Merge pull request `#52 <https://github.com/mikeferguson/robot_calibration/issues/52>`_ from mikeferguson/melodic-backport
  backport changes from melodic-devel branch
* fix OutrageousError
  This has apparently NEVER worked. The name that was being
  provided was the error block name, not the name of the
  parameter to limit.
* add to_rpy tool
  The YAML file output by calibration represents angles in
  the internal axis-magnitude notation. RPY tends to be
  easier for people to understand and visualize.
* add some comments to Camera3dModel
  In particular, a search for checkerboard should really turn up this
  important piece of code
* fix test build/warning issues on 18.04
* Merge pull request `#50 <https://github.com/mikeferguson/robot_calibration/issues/50>`_ from guilhermelawless/tf-buffer-member
  Make TF buffer a class member
* make TF buffer a class member
  Fixes `#48 <https://github.com/mikeferguson/robot_calibration/issues/48>`_. Sleeping to wait for TFs is no longer needed and was removed.
* Merge pull request `#47 <https://github.com/mikeferguson/robot_calibration/issues/47>`_ from guilhermelawless/fix-checkerboard-visualization
  Fix checkerboard visualization
* Merge pull request `#45 <https://github.com/mikeferguson/robot_calibration/issues/45>`_ from guilhermelawless/kinetic-devel
  Allow some time to get TFs in plane_finder
* fix checkerboard visualization msg
* allow some time to get TFs in plane_finder
* add a second error block test, that actually needs to converge
* fix parameter name in test, fork a second copy
* make sure solver ran in test
* fix cmake errors reported by buildfarm, update maintainer email
* attempt to fix test on kinetic
* clean up parameter loading, output printing, README
* convert camera_to_camera into plane_to_plane
* convert ground_plane_error into chain3d_to_plane_error
* convert camera3d_to_arm into chain3d_to_chain3d
* add param/residual checks to error block test
* merge GroundPlaneFinder into PlaneFinder
* make feature finders plugin-based, add tests
* Contributors: Guilherme Lawless, Michael Ferguson, saurabhbansal90

0.5.5 (2018-02-12)
------------------
* Merge pull request `#36 <https://github.com/mikeferguson/robot_calibration/issues/36>`_ from guilhermelawless/indigo-devel
  Fix broken OpenCV linking in ROS Kinetic
* Contributors: Guilherme Lawless, Michael Ferguson

0.5.4 (2018-01-20)
------------------
* only add observations when complete
* Adds plane calibration
* minor style fixes, remove outdated comments
* fix warning (`#28 <https://github.com/mikeferguson/robot_calibration/issues/28>`_)
* pick correct sensor in each error block
* use proper indices for multiple finders
* fix: don't append observations if finder has failed
* Contributors: Martin Günther, Michael Ferguson, Niharika Arora

0.5.3 (2016-07-18)
------------------
* add support for multiple finders in a given pose
* add support for ground plane calibration
* add parameter for camera_info_topic in depth camera capture module
* Contributors: Michael Ferguson, Niharika Arora

0.5.2 (2015-07-03)
------------------
* remove dependency on PCL
* cleanup naming of member variables
* fix centroid refinement, fixes `#20 <https://github.com/mikeferguson/robot_calibration/issues/20>`_
* Contributors: Michael Ferguson

0.5.1 (2015-07-01)
------------------
* store calibration output in unique file name
* better memory management in optimizer
* parameterize sensor names in finders
* checkerboard finder working on fetch
* Contributors: Michael Ferguson

0.5.0 (2015-06-23)
------------------
* add new CaptureConfig message for setting up samples
* update optimizer to handle new types of error blocks
* cleanup how we use the depth camera manager
* refactor how feature finders are loaded
* Contributors: Michael Ferguson

0.4.1 (2015-06-17)
------------------
* check distance to expected pose in tracker process()
* Contributors: Michael Ferguson

0.4.0 (2015-06-07)
------------------
* fix for multiple joint_state publishers, roll back async spinner changes
* output tracker status as image
* Contributors: Michael Ferguson

0.3.1 (2015-04-23)
------------------
* start async spinner earlier
* update how we sleep for better data capture
* Contributors: Michael Ferguson

0.3.0 (2015-04-22)
------------------
* process all callbacks in async spinner
* make waitForCloud consistent between feature detectors
* remove all calls to spinOnce in feature detectors, chain management
* exit if not ros::ok(), fixes `#12 <https://github.com/mikeferguson/robot_calibration/issues/12>`_
* do not capture if move failed, fixes `#14 <https://github.com/mikeferguson/robot_calibration/issues/14>`_
* publish point cloud for checkerboard detector
* Contributors: Michael Ferguson

0.2.2 (2015-04-12)
------------------
* add support for velocity scaling factor
* Contributors: Michael Ferguson

0.2.1 (2015-04-05)
------------------
* fix uninitialized variable
* test files should not use .launch extension
* fix error_block_test, closes `#11 <https://github.com/mikeferguson/robot_calibration/issues/11>`_
* fix issue with capture stalling
* Contributors: Michael Ferguson

0.2.0 (2015-03-16)
------------------
* enforce internal consistency between led features
* remove opencv window, add cloud in message option
* update how max error is handled
* extend messages to support multiple sensors
* implement ExtendedCameraInfo
* Contributors: Michael Ferguson

0.1.2 (2015-03-15)
------------------
* fix a number of warning
* enable use of moveit for planning between poses
* handle multiple joint_states publisher
* update checkerboard_finder config
* refactor led finder to use lots of parameters
* Contributors: Michael Ferguson

0.1.1 (2015-03-05)
------------------
* first release
* Contributors: Michael Ferguson
