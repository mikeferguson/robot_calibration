^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_calibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
