cmake_minimum_required(VERSION 3.5)
project(robot_calibration_msgs)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GripperLedCommand.action"
  "msg/CalibrationData.msg"
  "msg/CameraParameter.msg"
  "msg/CaptureConfig.msg"
  "msg/ExtendedCameraInfo.msg"
  "msg/Observation.msg"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
