if(CATKIN_ENABLE_TESTING)

find_package(rostest REQUIRED)

catkin_add_gtest(calibration_offset_parser_tests calibration_offset_parser_tests.cpp)
target_link_libraries(calibration_offset_parser_tests robot_calibration
                                                      ${Boost_LIBRARIES}
                                                      ${catkin_LIBRARIES}
                                                      ${orocos_kdl_LIBRARIES}
                                                      ${tinyxml_LIBRARIES})

catkin_add_gtest(chain_model_tests chain_model_tests.cpp)
target_link_libraries(chain_model_tests robot_calibration
                                     ${Boost_LIBRARIES}
                                     ${catkin_LIBRARIES}
                                     ${CERES_LIBRARIES}
                                     ${orocos_kdl_LIBRARIES})

catkin_add_gtest(rotation_tests rotation_tests.cpp)
target_link_libraries(rotation_tests robot_calibration
                                     ${Boost_LIBRARIES}
                                     ${catkin_LIBRARIES}
                                     ${CERES_LIBRARIES}
                                     ${orocos_kdl_LIBRARIES})

add_executable(camera_info_tests camera_info_tests.cpp)
add_dependencies(tests camera_info_tests)
target_link_libraries(camera_info_tests robot_calibration
                                  ${Boost_LIBRARIES}
                                  ${catkin_LIBRARIES}
                                  ${GTEST_LIBRARIES})

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

add_executable(error_block_tests2 error_block_tests2.cpp)
add_dependencies(tests error_block_tests2)
target_link_libraries(error_block_tests2 robot_calibration
                                         ${Boost_LIBRARIES}
                                         ${catkin_LIBRARIES}
                                         ${CERES_LIBRARIES}
                                         ${GTEST_LIBRARIES}
                                         ${orocos_kdl_LIBRARIES})

add_executable(feature_finder_loader_tests feature_finder_loader_tests.cpp)
add_dependencies(tests feature_finder_loader_tests)
target_link_libraries(feature_finder_loader_tests robot_calibration
                                                  ${Boost_LIBRARIES}
                                                  ${catkin_LIBRARIES}
                                                  ${GTEST_LIBRARIES})

add_executable(optimization_param_tests optimization_param_tests.cpp)
add_dependencies(tests optimization_param_tests)
target_link_libraries(optimization_param_tests robot_calibration
                                               ${Boost_LIBRARIES}
                                               ${catkin_LIBRARIES}
                                               ${GTEST_LIBRARIES})

add_rostest(camera_info_tests.test)
add_rostest(chain_manager_tests.test)
add_rostest(feature_finder_loader_tests.test)
add_rostest(optimization_param_tests.test)
add_rostest(error_block_tests.test)
add_rostest(error_block_tests2.test)

if(ENABLE_COVERAGE_TESTING)
  set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
  add_code_coverage(
    NAME ${PROJECT_NAME}_coverage_report
    DEPENDENCIES _run_tests_${PROJECT_NAME}
  )
endif()

endif()  # if CATKIN_ENABLE_TESTING
