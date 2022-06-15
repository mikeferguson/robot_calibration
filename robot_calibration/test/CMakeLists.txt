find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(optimization_offsets_tests optimization_offsets_tests.cpp)
target_link_libraries(optimization_offsets_tests robot_calibration
                                                 ${orocos_kdl_LIBRARIES})
ament_target_dependencies(optimization_offsets_tests ${dependencies})

ament_add_gtest(chain_model_tests chain_model_tests.cpp)
target_link_libraries(chain_model_tests robot_calibration
                                        ${CERES_LIBRARIES}
                                        ${orocos_kdl_LIBRARIES})
ament_target_dependencies(chain_model_tests ${dependencies})

ament_add_gtest(poses_from_yaml_tests poses_from_yaml_tests.cpp
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(poses_from_yaml_tests robot_calibration)
ament_target_dependencies(poses_from_yaml_tests ${dependencies})

ament_add_gtest(rotation_tests rotation_tests.cpp)
target_link_libraries(rotation_tests robot_calibration
                                     ${CERES_LIBRARIES}
                                     ${orocos_kdl_LIBRARIES})
ament_target_dependencies(rotation_tests ${dependencies})

ament_add_gtest_executable(camera_info_tests camera_info_tests.cpp)
target_link_libraries(camera_info_tests robot_calibration
                                        ${GTEST_LIBRARIES})
ament_target_dependencies(camera_info_tests ${dependencies})

ament_add_gtest_executable(chain_manager_tests chain_manager_tests.cpp)
target_link_libraries(chain_manager_tests robot_calibration
                                          ${GTEST_LIBRARIES})
ament_target_dependencies(chain_manager_tests ${dependencies})

ament_add_gtest_executable(error_block_tests error_block_tests.cpp)
target_link_libraries(error_block_tests robot_calibration
                                        ${CERES_LIBRARIES}
                                        ${GTEST_LIBRARIES}
                                        ${orocos_kdl_LIBRARIES})
ament_target_dependencies(error_block_tests ${dependencies})

ament_add_gtest_executable(error_block_tests2 error_block_tests2.cpp)
target_link_libraries(error_block_tests2 robot_calibration
                                         ${CERES_LIBRARIES}
                                         ${GTEST_LIBRARIES}
                                         ${orocos_kdl_LIBRARIES})
ament_target_dependencies(error_block_tests2 ${dependencies})

ament_add_gtest_executable(feature_finder_loader_tests feature_finder_loader_tests.cpp)
target_link_libraries(feature_finder_loader_tests robot_calibration ${GTEST_LIBRARIES})
ament_target_dependencies(feature_finder_loader_tests ${dependencies})

ament_add_gtest_executable(optimization_param_tests optimization_param_tests.cpp)
target_link_libraries(optimization_param_tests robot_calibration ${GTEST_LIBRARIES})
ament_target_dependencies(optimization_param_tests ${dependencies})

ament_add_test(camera_info_tests_launch
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/camera_info_tests_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:camera_info_tests>
    INFO_PUBLISHER=${CMAKE_CURRENT_SOURCE_DIR}/camera_info_publisher.py
)

ament_add_test(chain_manager_tests_launch
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/chain_manager_tests_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:chain_manager_tests>
    CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}
)

ament_add_test(error_block_tests_launch
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/error_block_tests_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:error_block_tests>
    CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}
)

ament_add_test(error_block_tests2_launch
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/error_block_tests2_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:error_block_tests2>
    CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}
)

ament_add_test(feature_finder_loader_test_launch
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/feature_finder_loader_tests_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:feature_finder_loader_tests>
    INFO_PUBLISHER=${CMAKE_CURRENT_SOURCE_DIR}/camera_info_publisher.py
    CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}
)

ament_add_test(optimization_param_tests_launch
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/optimization_param_tests_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:optimization_param_tests>
    CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}
)
