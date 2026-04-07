cmake_minimum_required(VERSION 3.5)
project(mcl_3dl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if($ENV{ROS_VERSION} EQUAL 1)
  message(STATUS "Building for ROS 1 (Noetic)")

  set(CATKIN_DEPENDS
    roscpp
    ros1_rclcpp_compat

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

    if(NOT "$ENV{ROS_DISTRO}" STREQUAL "melodic")
      find_package(roslint REQUIRED)
      roslint_cpp()
      roslint_add_test()
    endif()
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
  target_include_directories(mcl_3dl_engine SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
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
    target_compile_options(test_nd PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_pf test/src/test_pf.cpp)
    target_include_directories(test_pf PRIVATE include)
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
    target_include_directories(test_state_6dof SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(test_state_6dof rclcpp)
    target_compile_options(test_state_6dof PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_imu_measurement_model_gravity test/src/test_imu_measurement_model_gravity.cpp)
    target_include_directories(test_imu_measurement_model_gravity PRIVATE include)
    target_include_directories(test_imu_measurement_model_gravity SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(test_imu_measurement_model_gravity rclcpp)
    target_compile_options(test_imu_measurement_model_gravity PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_motion_prediction_model_differential_drive test/src/test_motion_prediction_model_differential_drive.cpp)
    target_include_directories(test_motion_prediction_model_differential_drive PRIVATE include)
    target_include_directories(test_motion_prediction_model_differential_drive SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(test_motion_prediction_model_differential_drive rclcpp)
    target_compile_options(test_motion_prediction_model_differential_drive PRIVATE ${UNIT_TEST_COMPILE_OPTIONS})

    ament_add_gtest(test_noise_generator test/src/test_noise_generator.cpp)
    target_include_directories(test_noise_generator PRIVATE include)
    target_include_directories(test_noise_generator SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
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
  endif()

  ament_auto_package()

else()
  message(FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
