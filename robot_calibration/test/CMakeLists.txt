find_package(rostest REQUIRED)

catkin_add_gtest(calibration_offset_parser_tests calibration_offset_parser_tests.cpp)
target_link_libraries(calibration_offset_parser_tests robot_calibration
                                                      ${Boost_LIBRARIES}
                                                      ${catkin_LIBRARIES}
                                                      ${orocos_kdl_LIBRARIES}
                                                      ${tinyxml_LIBRARIES})

catkin_add_gtest(rotation_tests rotation_tests.cpp)
target_link_libraries(rotation_tests robot_calibration
                                     ${Boost_LIBRARIES}
                                     ${catkin_LIBRARIES}
                                     ${CERES_LIBRARIES}
                                     ${orocos_kdl_LIBRARIES})

add_executable(chain_manager_tests chain_manager_tests.cpp)
add_dependencies(tests chain_manager_tests)
target_link_libraries(chain_manager_tests robot_calibration
                                          ${Boost_LIBRARIES}
                                          ${catkin_LIBRARIES}
                                          ${GTEST_LIBRARIES})

add_executable(error_block_tests error_block_tests.cpp)
add_dependencies(tests error_block_tests)
target_link_libraries(error_block_tests robot_calibration
                                        ${Boost_LIBRARIES}
                                        ${catkin_LIBRARIES}
                                        ${CERES_LIBRARIES}
                                        ${GTEST_LIBRARIES}
                                        ${orocos_kdl_LIBRARIES})

add_executable(optimization_param_tests optimization_param_tests.cpp)
add_dependencies(tests optimization_param_tests)
target_link_libraries(optimization_param_tests robot_calibration
                                               ${Boost_LIBRARIES}
                                               ${catkin_LIBRARIES}
                                               ${GTEST_LIBRARIES})

add_rostest(chain_manager_tests.test)
add_rostest(optimization_param_tests.test)
add_rostest(error_block_tests.test)
