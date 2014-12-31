catkin_add_gtest(test_error_blocks test_error_blocks.cpp
                                   ../src/calibration_offset_parser.cpp
                                   ../src/models.cpp)
target_link_libraries(test_error_blocks ${Boost_LIBRARIES}
                                        ${catkin_LIBRARIES}
                                        ${CERES_LIBRARIES}
                                        ${orocos_kdl_LIBRARIES})

catkin_add_gtest(test_calibration_offset_parser test_calibration_offset_parser.cpp
                                  ../src/calibration_offset_parser.cpp
                                  ../src/models.cpp)
target_link_libraries(test_calibration_offset_parser ${Boost_LIBRARIES}
                                                     ${catkin_LIBRARIES}
                                                     ${orocos_kdl_LIBRARIES}
                                                     ${tinyxml_LIBRARIES})

catkin_add_gtest(test_rotations test_rotations.cpp
                                ../src/calibration_offset_parser.cpp
                                ../src/models.cpp)
target_link_libraries(test_rotations ${Boost_LIBRARIES}
                                     ${catkin_LIBRARIES}
                                     ${CERES_LIBRARIES}
                                     ${orocos_kdl_LIBRARIES})
