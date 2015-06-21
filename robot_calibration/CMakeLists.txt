cmake_minimum_required(VERSION 2.8.3)
project(robot_calibration)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(Boost REQUIRED system thread)
find_package(PCL REQUIRED)

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
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    camera_calibration_parsers
    cv_bridge
    geometry_msgs
    kdl_parser
    moveit_msgs
    nav_msgs
    pcl_ros
    robot_calibration_msgs
    rosbag
    roscpp
    sensor_msgs
    std_msgs
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
    pcl_ros
    robot_calibration_msgs
    rosbag
    roscpp
    sensor_msgs
    std_msgs
  DEPENDS
    Boost
    Ceres
    orocos_kdl
  LIBRARIES
    robot_calibration
)

include_directories(include ${Boost_INCLUDE_DIRS}
                            ${catkin_INCLUDE_DIRS}
                            ${CERES_INCLUDES}
                            ${PCL_INCLUDE_DIRS}
                            ${orocos_kdl_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS}) # this is a hack, will eventually be unneeded once orocos-kdl is fixed

add_executable(calibrate_base src/calibrate_base.cpp)
target_link_libraries(calibrate_base ${Boost_LIBRARIES}
                                     ${catkin_LIBRARIES})

add_library(robot_calibration src/calibration_offset_parser.cpp
                              src/chain_manager.cpp
                              src/checkerboard_finder.cpp
                              src/feature_finder.cpp
                              src/led_finder.cpp
                              src/models.cpp
                              src/optimization_params.cpp
                              src/optimizer.cpp)
target_link_libraries(robot_calibration ${Boost_LIBRARIES}
                                        ${catkin_LIBRARIES}
                                        ${CERES_LIBRARIES}
                                        ${tinyxml_LIBRARIES}
                                        ${orocos_kdl_LIBRARIES})
add_dependencies(robot_calibration robot_calibration_msgs_gencpp)

add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate robot_calibration
                                ${Boost_LIBRARIES}
                                ${catkin_LIBRARIES}
                                ${CERES_LIBRARIES}
                                ${tinyxml_LIBRARIES}
                                ${orocos_kdl_LIBRARIES})
add_dependencies(calibrate robot_calibration)

add_subdirectory(test)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS calibrate calibrate_base robot_calibration
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})