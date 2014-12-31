find_package(rostest REQUIRED)

catkin_add_gtest(error_block_tests error_block_tests.cpp
                                   ../src/calibration_offset_parser.cpp
                                   ../src/models.cpp)
target_link_libraries(error_block_tests ${Boost_LIBRARIES}
                                        ${catkin_LIBRARIES}
                                        ${CERES_LIBRARIES}
                                        ${orocos_kdl_LIBRARIES})

catkin_add_gtest(calibration_offset_parser_tests calibration_offset_parser_tests.cpp
                                  ../src/calibration_offset_parser.cpp
                                  ../src/models.cpp)
target_link_libraries(calibration_offset_parser_tests ${Boost_LIBRARIES}
                                                      ${catkin_LIBRARIES}
                                                      ${orocos_kdl_LIBRARIES}
                                                      ${tinyxml_LIBRARIES})

catkin_add_gtest(rotation_tests rotation_tests.cpp
                                ../src/calibration_offset_parser.cpp
                                ../src/models.cpp)
target_link_libraries(rotation_tests ${Boost_LIBRARIES}
                                     ${catkin_LIBRARIES}
                                     ${CERES_LIBRARIES}
                                     ${orocos_kdl_LIBRARIES})

add_executable(chain_manager_tests chain_manager_tests.cpp
                                   ../src/chain_manager.cpp)
add_dependencies(tests chain_manager_tests)
target_link_libraries(chain_manager_tests ${Boost_LIBRARIES}
                                          ${catkin_LIBRARIES}
                                          ${GTEST_LIBRARIES})

add_rostest(chain_manager_tests.launch)