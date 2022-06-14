cmake_minimum_required(VERSION 3.5)
project(robot_calibration)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_library(tinyxml_library tinyxml)
if (tinyxml_library)
  message (STATUS "Looking for libtinyxml - found")
  set(tinyxml_LIBRARIES ${tinyxml_library})
endif ()
find_path(tinyxml_include_dirs NAMES tinyxml.h PATH_SUFFIXES tinyxml)
if (NOT tinyxml_include_dirs)
   message (STATUS "Looking for tinyxml/tinyxml.hpp or tinyxml/tinyxml.h - not found.")
endif ()

find_package(orocos_kdl REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(Ceres REQUIRED)
find_package(control_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_calibration_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    ${CERES_INCLUDES}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS}) # this is a hack, will eventually be unneeded once orocos-kdl is fixed

set(dependencies
  camera_calibration_parsers
  control_msgs
  cv_bridge
  geometry_msgs
  geometric_shapes
  kdl_parser
  moveit_msgs
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  robot_calibration_msgs
  rosbag2_cpp
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
)

add_library(robot_calibration SHARED
  src/base_calibration.cpp
  src/models.cpp
  src/optimization/ceres_optimizer.cpp
  src/optimization/export.cpp 
  src/optimization/offsets.cpp
  src/optimization/params.cpp
  src/util/capture_manager.cpp
  src/util/capture_poses.cpp
  src/util/chain_manager.cpp
  src/util/mesh_loader.cpp
)
target_link_libraries(robot_calibration
  ${Boost_LIBRARIES}
  ${CERES_LIBRARIES}
  ${tinyxml_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
ament_target_dependencies(robot_calibration
  ${dependencies}
)

add_library(robot_calibration_feature_finders
  src/finders/checkerboard_finder.cpp
  src/finders/led_finder.cpp
  src/finders/plane_finder.cpp
  src/finders/robot_finder.cpp
  src/finders/scan_finder.cpp
)
target_link_libraries(robot_calibration_feature_finders
  robot_calibration
  ${Boost_LIBRARIES}
  ${tinyxml_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${OpenCV_LIBS}
)
ament_target_dependencies(robot_calibration_feature_finders
  ${dependencies}
)

add_executable(calibrate src/nodes/calibrate.cpp)
target_link_libraries(calibrate
  robot_calibration
  ${Boost_LIBRARIES}
  ${CERES_LIBRARIES}
  ${tinyxml_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${OpenCV_LIBS}
)
ament_target_dependencies(calibrate
  ${dependencies}
)

add_executable(base_calibration_node src/nodes/base_calibration.cpp)
target_link_libraries(base_calibration_node
  robot_calibration
  ${Boost_LIBRARIES}
)

add_executable(magnetometer_calibration src/nodes/magnetometer_calibration.cpp)
target_link_libraries(magnetometer_calibration
  ${CERES_LIBRARIES}
)
ament_target_dependencies(magnetometer_calibration
  ${dependencies}
)

add_executable(to_rpy src/nodes/to_rpy.cpp)
target_link_libraries(to_rpy
  robot_calibration
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

add_executable(viz src/nodes/viz.cpp)
target_link_libraries(viz
  robot_calibration
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

add_executable(viz_mesh src/nodes/viz_mesh.cpp)
target_link_libraries(viz_mesh
  robot_calibration
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(DIRECTORY include/ DESTINATION include)

install(TARGETS
  base_calibration_node
  calibrate
  magnetometer_calibration
  robot_calibration
  robot_calibration_feature_finders
  to_rpy
  viz
  viz_mesh
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/robot_calibration
)

pluginlib_export_plugin_description_file(robot_calibration robot_calibration.xml)

ament_export_include_directories(include)
ament_export_libraries(robot_calibration robot_calibration_feature_finders)
ament_export_dependencies(${dependencies})
ament_package()
