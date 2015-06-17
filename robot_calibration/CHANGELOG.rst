^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_calibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
