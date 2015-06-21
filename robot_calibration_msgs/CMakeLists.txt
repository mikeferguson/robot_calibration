cmake_minimum_required(VERSION 2.8.3)
project(robot_calibration_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    geometry_msgs
    message_generation
    sensor_msgs
    std_msgs
)

add_action_files(
  FILES
    GripperLedCommand.action
)

add_message_files(
  FILES
    CalibrationData.msg
    CameraParameter.msg
    CaptureConfig.msg
    ExtendedCameraInfo.msg
    Observation.msg
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
    sensor_msgs
    std_msgs
)
