# Robot Calibration
See ROS wiki for usage. See doxygen documentation for source details.

# Future improvements:
The two major possible improvements are improving the led finder, and testing and possibly
fixing checkerboard-based calibration (especially for vendors who maybe use a custom gripper).
The led finder may be improved in several ways:

 * Rather than centroid finding, perhaps do a best fit of a circle.
 * Possibly use just the presence/absence of points to find LEDs in the depth data only, averting
   possible errors where the RGB and depth are not fully aligned. In this case it might be possible
   to also then improve the alignment between RGB and depth data by finding the same points both
   in the depth and rgb images.

Also, it may be worthwhile to create a better calibration_poses.bag, with more and better poses.
It should be possible to capture a huge number of possible poses and then run a simulation to
determine which poses to keep to get the best convergence.

There are also some minor items:

 * Add test and backup options to robot_calibration tool.
 * Remove PCL dependency by refactoring to use sensor_msgs::PointCloud2Iterator.