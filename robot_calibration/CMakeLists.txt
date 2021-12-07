cmake_minimum_required(VERSION 2.8.3)
project(robot_calibration)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost REQUIRED system thread)

find_library(tinyxml_library tinyxml)
if (tinyxml_library)
  message (STATUS "Looking for libtinyxml - found")
  set(tinyxml_LIBRARIES ${tinyxml_library})
endif ()
find_path(tinyxml_include_dirs NAMES tinyxml.h PATH_SUFFIXES tinyxml)
if (NOT tinyxml_include_dirs)
   message (STATUS "Looking for tinyxml/tinyxml.hpp or tinyxml/tinyxml.h - not found.")
endif ()

find_package(orocos_kdl)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    camera_calibration_parsers
    cv_bridge
    geometry_msgs
    kdl_parser
    moveit_msgs
    nav_msgs
    pluginlib
    robot_calibration_msgs
    rosbag
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
    backward_ros
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    camera_calibration_parsers
    cv_bridge
    geometry_msgs
    kdl_parser
    moveit_msgs
    nav_msgs
    pluginlib
    robot_calibration_msgs
    rosbag
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2_geometry_msgs
    tf2_ros
  DEPENDS
    Boost
    orocos_kdl
  LIBRARIES
    robot_calibration
    robot_calibration_feature_finders
)

if (CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

include_directories(include ${Boost_INCLUDE_DIRS}
                            ${catkin_INCLUDE_DIRS}
                            ${CERES_INCLUDES}
                            ${orocos_kdl_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS}) # this is a hack, will eventually be unneeded once orocos-kdl is fixed

add_executable(calibrate_base src/calibrate_base.cpp)
target_link_libraries(calibrate_base ${Boost_LIBRARIES}
                                     ${catkin_LIBRARIES})

add_executable(magnetometer_calibration src/magnetometer_calibration.cpp)
target_link_libraries(magnetometer_calibration
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES})

add_library(robot_calibration src/calibration_offset_parser.cpp
                              src/chain_manager.cpp
                              src/models.cpp
                              src/optimization_params.cpp
                              src/optimizer.cpp)
target_link_libraries(robot_calibration ${Boost_LIBRARIES}
                                        ${catkin_LIBRARIES}
                                        ${CERES_LIBRARIES}
                                        ${tinyxml_LIBRARIES}
                                        ${orocos_kdl_LIBRARIES})
add_dependencies(robot_calibration robot_calibration_msgs_generate_messages_cpp)

add_library(robot_calibration_feature_finders src/finders/checkerboard_finder.cpp
                                              src/finders/led_finder.cpp
                                              src/finders/plane_finder.cpp)
target_link_libraries(robot_calibration_feature_finders robot_calibration
                                                        ${Boost_LIBRARIES}
                                                        ${catkin_LIBRARIES}
                                                        ${tinyxml_LIBRARIES}
                                                        ${orocos_kdl_LIBRARIES}
                                                        ${OpenCV_LIBS})
add_dependencies(robot_calibration_feature_finders robot_calibration_msgs_generate_messages_cpp)

add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate robot_calibration
                                ${Boost_LIBRARIES}
                                ${catkin_LIBRARIES}
                                ${CERES_LIBRARIES}
                                ${tinyxml_LIBRARIES}
                                ${orocos_kdl_LIBRARIES}
                                ${OpenCV_LIBS})
add_dependencies(calibrate robot_calibration)

add_library(capture_features_lib src/details/capture_features_lib.cpp)
target_link_libraries(capture_features_lib
                                ${Boost_LIBRARIES}
                                ${catkin_LIBRARIES})
add_executable(capture_features src/capture_features.cpp)
target_link_libraries(capture_features 
                                capture_features_lib
                                robot_calibration
                                ${Boost_LIBRARIES}
                                ${catkin_LIBRARIES}
                                ${CERES_LIBRARIES}
                                ${tinyxml_LIBRARIES}
                                ${orocos_kdl_LIBRARIES}
                                ${OpenCV_LIBS})
add_dependencies(capture_features robot_calibration)

add_executable(optimize_parameters src/optimize_parameters.cpp)
target_link_libraries(optimize_parameters robot_calibration
                                ${Boost_LIBRARIES}
                                ${catkin_LIBRARIES}
                                ${CERES_LIBRARIES}
                                ${tinyxml_LIBRARIES}
                                ${orocos_kdl_LIBRARIES}
                                ${OpenCV_LIBS})
add_dependencies(optimize_parameters robot_calibration)

add_executable(to_rpy src/tools/to_rpy.cpp)
target_link_libraries(to_rpy robot_calibration
                             ${Boost_LIBRARIES}
                             ${catkin_LIBRARIES}
                             ${orocos_kdl_LIBRARIES})
add_dependencies(to_rpy robot_calibration)

add_executable(viz src/tools/viz.cpp)
target_link_libraries(viz robot_calibration
                             ${Boost_LIBRARIES}
                             ${catkin_LIBRARIES}
                             ${orocos_kdl_LIBRARIES})
add_dependencies(viz robot_calibration)

add_subdirectory(test)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS
	calibrate
	calibrate_base
  capture_features
  capture_features_lib
  optimize_parameters
	robot_calibration
	robot_calibration_feature_finders
	to_rpy
	viz
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(FILES robot_calibration.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
