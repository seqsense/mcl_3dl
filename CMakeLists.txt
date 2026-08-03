cmake_minimum_required(VERSION 3.5)
project(mcl_3dl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if($ENV{ROS_VERSION} EQUAL 1)
  message(STATUS "Building for ROS 1 (Noetic)")

  set(CATKIN_DEPENDS
    roscpp
    sq_ros1_rclcpp_compat

    pcl_ros
    tf2
    tf2_geometry_msgs
    tf2_ros
    tf2_sensor_msgs

    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    mcl_3dl_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    std_srvs
    visualization_msgs
  )

  find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
  find_package(Eigen3 REQUIRED)
  find_package(PCL 1.8.0 REQUIRED
    COMPONENTS
      core
      filters
      kdtree
  )
  find_package(Boost REQUIRED COMPONENTS chrono)

  generate_dynamic_reconfigure_options(
    cfg/MCL3DLParams.cfg
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES mcl_3dl_engine
    CATKIN_DEPENDS ${CATKIN_DEPENDS}
  )

  include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIR})
  add_definitions(${PCL_DEFINITIONS})


  # Workaround for debian stretch build (https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=894656)
  list(REMOVE_ITEM PCL_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt"
  )

  # Workaround for the bug in PCL<1.8.1 (https://github.com/PointCloudLibrary/pcl/issues/1406)
  remove_definitions(-DDISABLE_LIBUSB-1.0)

  # Workaround for PCL-1.7 problem on C++11 (https://github.com/PointCloudLibrary/pcl/issues/619)
  # Debug build without optimization causes Segmentation fault.
  # Set default build type to Release and add -O1 to CXX_FLAGS_DEBUG.
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")

  # Binary installed pcl provided by Linux distro is built with -march=native
  # which causes a lot of compatibility problems.
  # Define PCL_NO_PRECOMPILE to disable using the binary version.
  add_definitions(-DPCL_NO_PRECOMPILE)


  add_library(mcl_3dl_engine
    src/cloud_accum.cpp
    src/lidar_measurement_model_beam.cpp
    src/lidar_measurement_model_likelihood.cpp
    src/mcl_3dl_engine.cpp
    src/parameters.cpp
    src/point_types.cpp
  )
  target_link_libraries(mcl_3dl_engine ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
  add_dependencies(mcl_3dl_engine ${PROJECT_NAME}_gencfg)

  add_executable(mcl_3dl
    src/ros1_mcl_3dl.cpp
  )
  target_link_libraries(mcl_3dl mcl_3dl_engine ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
  add_dependencies(mcl_3dl ${PROJECT_NAME}_gencfg)


  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(rosunit REQUIRED)
    add_subdirectory(test)
    # roslint is intentionally not used: style checking for this hybrid package
    # is unified on the ROS 2 side via ament_lint_auto (see the ROS 2 block).
  endif()


  install(TARGETS mcl_3dl_engine mcl_3dl
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
  install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

elseif($ENV{ROS_VERSION} EQUAL 2)
  message(STATUS "Building for ROS 2 (Jazzy)")

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  find_package(Eigen3 REQUIRED)
  find_package(PCL 1.12.0 REQUIRED
    COMPONENTS
      core
      filters
      kdtree
  )
  find_package(Boost REQUIRED COMPONENTS chrono)

  add_definitions(-DPCL_NO_PRECOMPILE)

  ament_auto_add_library(mcl_3dl_engine SHARED
    src/cloud_accum.cpp
    src/lidar_measurement_model_beam.cpp
    src/lidar_measurement_model_likelihood.cpp
    src/mcl_3dl_engine.cpp
    src/parameters.cpp
    src/point_types.cpp
  )
  target_include_directories(mcl_3dl_engine SYSTEM PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(mcl_3dl_engine ${Boost_LIBRARIES} ${PCL_LIBRARIES})

  # ROS2 component node
  ament_auto_add_library(mcl_3dl_node SHARED
    src/ros2_mcl_3dl_node.cpp
  )
  target_include_directories(mcl_3dl_node SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(mcl_3dl_node mcl_3dl_engine ${PCL_LIBRARIES})
  rclcpp_components_register_node(
    mcl_3dl_node PLUGIN "mcl_3dl::MCL3dlNode" EXECUTABLE mcl_3dl_exec)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # ROS 1 only sources are not built by this ROS 2 package; keep them out of
    # the linters (absolute paths, as required by AMENT_LINT_AUTO_FILE_EXCLUDE).
    set(AMENT_LINT_AUTO_FILE_EXCLUDE
      ${CMAKE_CURRENT_SOURCE_DIR}/include/mcl_3dl_compat/compatibility.h
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ros1_mcl_3dl.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/compare_pose.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/compare_tf.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/performance_raycast.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_beam_label.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_beam_likelihood.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_debug_output.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_expansion_resetting.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_global_localization.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_landmark.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_mcl_3dl_compat.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_parameters.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_transform_failure.cpp)
    # CPP_FORMATTER=clang_format: exclude uncrustify, keep clang_format + cpplint.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
    # The upstream BSD-3-Clause headers here carry an extra "All rights
    # reserved." line, so ament_copyright reports license=<unknown> for every
    # file. The linter is excluded rather than rewriting the license text.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)

    set(UNIT_TEST_COMPILE_OPTIONS -g -O0)

    ament_add_gtest(test_chunked_kdtree test/src/test_chunked_kdtree.cpp)
    target_include_directories(test_chunked_kdtree PRIVATE include)
    target_include_directories(test_chunked_kdtree SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    target_link_libraries(test_chunked_kdtree ${PCL_LIBRARIES})
    target_compile_options(test_chunked_kdtree PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_cloud_accum test/src/test_cloud_accum.cpp src/cloud_accum.cpp)
    target_include_directories(test_cloud_accum PRIVATE include)
    ament_target_dependencies(test_cloud_accum rclcpp sensor_msgs)
    target_compile_options(test_cloud_accum PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_filter test/src/test_filter.cpp)
    target_include_directories(test_filter PRIVATE include)
    target_compile_options(test_filter PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_nd test/src/test_nd.cpp)
    target_include_directories(test_nd PRIVATE include)
    target_include_directories(test_nd SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
    target_compile_options(test_nd PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_pf test/src/test_pf.cpp)
    target_include_directories(test_pf PRIVATE include)
    target_include_directories(test_pf SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
    target_compile_options(test_pf PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_point_types test/src/test_point_types.cpp)
    target_include_directories(test_point_types PRIVATE include)
    target_include_directories(test_point_types SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    target_link_libraries(test_point_types ${PCL_LIBRARIES})
    target_compile_options(test_point_types PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_quat test/src/test_quat.cpp)
    target_include_directories(test_quat PRIVATE include)
    target_compile_options(test_quat PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_vec3 test/src/test_vec3.cpp)
    target_include_directories(test_vec3 PRIVATE include)
    target_compile_options(test_vec3 PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_state_6dof test/src/test_state_6dof.cpp)
    target_include_directories(test_state_6dof PRIVATE include)
    target_include_directories(test_state_6dof SYSTEM PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
    ament_target_dependencies(test_state_6dof rclcpp)
    target_compile_options(test_state_6dof PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_imu_measurement_model_gravity test/src/test_imu_measurement_model_gravity.cpp)
    target_include_directories(test_imu_measurement_model_gravity PRIVATE include)
    target_include_directories(test_imu_measurement_model_gravity SYSTEM PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
    ament_target_dependencies(test_imu_measurement_model_gravity rclcpp)
    target_compile_options(test_imu_measurement_model_gravity PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_motion_prediction_model_differential_drive test/src/test_motion_prediction_model_differential_drive.cpp)
    target_include_directories(test_motion_prediction_model_differential_drive PRIVATE include)
    target_include_directories(test_motion_prediction_model_differential_drive SYSTEM PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
    ament_target_dependencies(test_motion_prediction_model_differential_drive rclcpp)
    target_compile_options(test_motion_prediction_model_differential_drive PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_noise_generator test/src/test_noise_generator.cpp)
    target_include_directories(test_noise_generator PRIVATE include)
    target_include_directories(test_noise_generator SYSTEM PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
    ament_target_dependencies(test_noise_generator rclcpp)
    target_compile_options(test_noise_generator PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_raycast test/src/test_raycast.cpp)
    target_include_directories(test_raycast PRIVATE include)
    target_include_directories(test_raycast SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    target_link_libraries(test_raycast ${PCL_LIBRARIES})
    target_compile_options(test_raycast PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_raycast_dda test/src/test_raycast_dda.cpp)
    target_include_directories(test_raycast_dda PRIVATE include)
    target_include_directories(test_raycast_dda SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    target_link_libraries(test_raycast_dda ${PCL_LIBRARIES})
    target_compile_options(test_raycast_dda PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_point_cloud_random_sampler test/src/test_point_cloud_random_sampler.cpp)
    target_include_directories(test_point_cloud_random_sampler PRIVATE include)
    target_include_directories(test_point_cloud_random_sampler SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(test_point_cloud_random_sampler rclcpp pcl_conversions)
    target_link_libraries(test_point_cloud_random_sampler ${PCL_LIBRARIES})
    target_compile_options(test_point_cloud_random_sampler PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_point_cloud_random_sampler_with_normal test/src/test_point_cloud_random_sampler_with_normal.cpp)
    target_include_directories(test_point_cloud_random_sampler_with_normal PRIVATE include)
    target_include_directories(test_point_cloud_random_sampler_with_normal SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(test_point_cloud_random_sampler_with_normal rclcpp pcl_conversions sensor_msgs)
    target_link_libraries(test_point_cloud_random_sampler_with_normal ${PCL_LIBRARIES})
    target_compile_options(test_point_cloud_random_sampler_with_normal PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    # --- launch_testing integration tests ---
    find_package(launch_testing_ament_cmake REQUIRED)

    # beam_likelihood (pure gtest, no node interaction needed)
    ament_add_gtest_executable(ros2_test_beam_likelihood
      test/src/ros2_test_beam_likelihood.cpp
      src/lidar_measurement_model_beam.cpp
      src/point_types.cpp
    )
    target_include_directories(ros2_test_beam_likelihood PRIVATE include)
    target_include_directories(ros2_test_beam_likelihood SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(ros2_test_beam_likelihood rclcpp pcl_conversions sensor_msgs)
    target_link_libraries(ros2_test_beam_likelihood ${PCL_LIBRARIES})
    target_compile_options(ros2_test_beam_likelihood PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})
    install(TARGETS ros2_test_beam_likelihood DESTINATION lib/${PROJECT_NAME})
    add_launch_test(test/tests/test_beam_likelihood_launch.py TIMEOUT 60)

    # debug_output
    ament_add_gtest_executable(ros2_test_debug_output test/src/ros2_test_debug_output.cpp)
    target_include_directories(ros2_test_debug_output PRIVATE include)
    ament_target_dependencies(ros2_test_debug_output
      rclcpp nav_msgs sensor_msgs tf2 tf2_geometry_msgs)
    target_compile_options(ros2_test_debug_output PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})
    install(TARGETS ros2_test_debug_output DESTINATION lib/${PROJECT_NAME})
    add_launch_test(test/tests/test_debug_output_launch.py TIMEOUT 60)

    # expansion_resetting
    ament_add_gtest_executable(ros2_test_expansion_resetting test/src/ros2_test_expansion_resetting.cpp)
    target_include_directories(ros2_test_expansion_resetting PRIVATE include)
    ament_target_dependencies(ros2_test_expansion_resetting
      rclcpp geometry_msgs mcl_3dl_msgs nav_msgs sensor_msgs std_srvs tf2 tf2_geometry_msgs)
    target_compile_options(ros2_test_expansion_resetting PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})
    install(TARGETS ros2_test_expansion_resetting DESTINATION lib/${PROJECT_NAME})
    add_launch_test(test/tests/test_expansion_resetting_launch.py TIMEOUT 90)

    # beam_label
    ament_add_gtest_executable(ros2_test_beam_label test/src/ros2_test_beam_label.cpp)
    target_include_directories(ros2_test_beam_label PRIVATE include)
    target_include_directories(ros2_test_beam_label SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(ros2_test_beam_label
      rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs)
    target_compile_options(ros2_test_beam_label PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})
    install(TARGETS ros2_test_beam_label DESTINATION lib/${PROJECT_NAME})
    add_launch_test(test/tests/test_beam_label_launch.py TIMEOUT 60)

    # landmark
    ament_add_gtest_executable(ros2_test_landmark test/src/ros2_test_landmark.cpp)
    target_include_directories(ros2_test_landmark PRIVATE include)
    ament_target_dependencies(ros2_test_landmark
      rclcpp geometry_msgs)
    target_compile_options(ros2_test_landmark PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})
    install(TARGETS ros2_test_landmark DESTINATION lib/${PROJECT_NAME})
    add_launch_test(test/tests/test_landmark_launch.py TIMEOUT 90)

    # transform_failure
    ament_add_gtest_executable(ros2_test_transform_failure test/src/ros2_test_transform_failure.cpp)
    target_include_directories(ros2_test_transform_failure PRIVATE include)
    ament_target_dependencies(ros2_test_transform_failure
      rclcpp geometry_msgs sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
    target_compile_options(ros2_test_transform_failure PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})
    install(TARGETS ros2_test_transform_failure DESTINATION lib/${PROJECT_NAME})
    add_launch_test(test/tests/test_transform_failure_launch.py TIMEOUT 60)

    # global_localization
    ament_add_gtest_executable(ros2_test_global_localization test/src/ros2_test_global_localization.cpp)
    target_include_directories(ros2_test_global_localization PRIVATE include)
    ament_target_dependencies(ros2_test_global_localization
      rclcpp geometry_msgs mcl_3dl_msgs nav_msgs sensor_msgs std_srvs tf2 tf2_geometry_msgs)
    target_compile_options(ros2_test_global_localization PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})
    install(TARGETS ros2_test_global_localization DESTINATION lib/${PROJECT_NAME})
    add_launch_test(test/tests/test_global_localization_launch.py TIMEOUT 180)

    # parameters
    ament_add_gtest_executable(ros2_test_parameters test/src/ros2_test_parameters.cpp)
    target_include_directories(ros2_test_parameters PRIVATE include)
    ament_target_dependencies(ros2_test_parameters rclcpp rcl_interfaces)
    target_compile_options(ros2_test_parameters PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})
    install(TARGETS ros2_test_parameters DESTINATION lib/${PROJECT_NAME})
    add_launch_test(test/tests/test_parameters_launch.py TIMEOUT 30)

    # ----- Heavy bag-replay tests (gated by MCL_3DL_EXTRA_TESTS) -----
    # ROS 2 port of localization_rostest.test.in / tf_rostest.test.in.
    # Requires the short_test3 bag (already converted to rosbag2 .db3) and
    # short_test_ref.topic (YAML dump of nav_msgs/Path) to be present under
    # test/data/. The bag is ~230MB so we intentionally do not download it
    # automatically; place the files manually or override at run time via
    # the MCL_3DL_TEST_DATA_DIR environment variable.
    if(MCL_3DL_EXTRA_TESTS OR "$ENV{MCL_3DL_EXTRA_TESTS}" STREQUAL "ON")
      find_package(yaml_cpp_vendor QUIET)
      if(NOT yaml_cpp_vendor_FOUND)
        find_package(yaml-cpp REQUIRED)
      endif()

      ament_add_gtest_executable(ros2_compare_pose test/src/ros2_compare_pose.cpp)
      target_include_directories(ros2_compare_pose PRIVATE include)
      ament_target_dependencies(ros2_compare_pose
        rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs)
      target_link_libraries(ros2_compare_pose yaml-cpp)
      target_compile_options(ros2_compare_pose PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})
      install(TARGETS ros2_compare_pose DESTINATION lib/${PROJECT_NAME})

      ament_add_gtest_executable(ros2_compare_tf test/src/ros2_compare_tf.cpp)
      target_include_directories(ros2_compare_tf PRIVATE include)
      ament_target_dependencies(ros2_compare_tf
        rclcpp geometry_msgs tf2 tf2_geometry_msgs tf2_ros)
      target_compile_options(ros2_compare_tf PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})
      install(TARGETS ros2_compare_tf DESTINATION lib/${PROJECT_NAME})

      # Install the bag and ref Path file to share/ so launch.py can find
      # them via ament_index. Override at run time with MCL_3DL_TEST_DATA_DIR
      # if you want to point at an out-of-tree copy.
      if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/data/short_test3"
          AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/data/short_test_ref.topic")
        install(DIRECTORY test/data/short_test3
                DESTINATION share/${PROJECT_NAME}/test_data)
        install(FILES test/data/short_test_ref.topic
                DESTINATION share/${PROJECT_NAME}/test_data)
        add_launch_test(test/tests/test_localization_launch.py TIMEOUT 420)
        add_launch_test(test/tests/test_tf_launch.py TIMEOUT 150)
      else()
        message(WARNING
          "MCL_3DL_EXTRA_TESTS is ON but test/data/short_test3 or "
          "short_test_ref.topic is missing under "
          "${CMAKE_CURRENT_SOURCE_DIR}/test/data/. Skipping bag-replay tests. "
          "Download short_test3.bag and short_test_ref.topic from "
          "https://openspur.org/~atsushi.w/dataset/mcl_3dl/ , then convert "
          "the bag to rosbag2 (e.g. `rosbags-convert --src short_test3.bag "
          "--dst short_test3 --dst-version 4 --src-typestore ros1_noetic "
          "--dst-typestore ros2_humble`) and place the resulting "
          "short_test3/ directory and short_test_ref.topic under the path "
          "above.")
      endif()
    else()
      message(STATUS
        "mcl_3dl: bag-replay tests skipped. Pass -DMCL_3DL_EXTRA_TESTS=ON or "
        "set MCL_3DL_EXTRA_TESTS=ON in the environment to enable them.")
    endif()
  endif()

  ament_auto_package()

else()
  message(FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
