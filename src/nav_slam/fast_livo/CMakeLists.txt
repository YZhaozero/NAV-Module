cmake_minimum_required(VERSION 3.10)
project(fast_livo)

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set common compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Specific settings for Debug build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Detect CPU architecture
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Distribution PCL/Eigen binaries use the baseline SIMD ABI. Enabling native
# AVX here changes Eigen's aligned allocation/free contract across that boundary.
option(FAST_LIVO_NATIVE_ARCH "Enable native CPU flags (requires matching dependencies)" OFF)
if(FAST_LIVO_NATIVE_ARCH)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native")
  else()
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
  endif()
else()
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
endif()

# Define project root directory
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Detect CPU core count for potential multithreading optimization
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# Set the number of cores for multithreading
if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
else()
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# Check for OpenMP support
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
else()
  message(STATUS "OpenMP not found, proceeding without it")
endif()

# Check for mimalloc support
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()

# Find ament and required dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "srv/SaveMap.srv")
rosidl_get_typesupport_target(nav_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
# ament 版 Sophus (ros-humble-sophus) 只导出 Sophus::Sophus target, 不设 Sophus_INCLUDE_DIRS
if(NOT Sophus_INCLUDE_DIRS AND TARGET Sophus::Sophus)
  get_target_property(Sophus_INCLUDE_DIRS Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
endif()
find_package(Boost REQUIRED COMPONENTS thread date_time)
find_package(fmt REQUIRED)
find_package(SQLite3 REQUIRED)

# Include directories for dependencies
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${vikit_common_INCLUDE_DIRS}
  ${vikit_ros_INCLUDE_DIRS}
  include
)

set(dependencies
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  cv_bridge
  vikit_common
  vikit_ros
  image_transport
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
)

set(COMMON_DEPENDENCIES OpenMP::OpenMP_CXX fmt::fmt)

# link_directories(${COMMON_DEPENDENCIES}
#   ${vikit_common_LIBRARIES}/libvikit_common.so
#   ${vikit_ros_LIBRARIES}/libvikit_ros.so
# )

# Add libraries
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp src/navigation_output.cpp)
add_library(utils src/utils.cpp)
add_library(rgb_projection src/rgb_projection_snapshot.cpp)
target_link_libraries(rgb_projection vikit_common::vikit_common ${OpenCV_LIBRARIES})

ament_target_dependencies(vio ${dependencies} )
ament_target_dependencies(lio ${dependencies})
ament_target_dependencies(pre ${dependencies})
ament_target_dependencies(imu_proc ${dependencies})
ament_target_dependencies(laser_mapping ${dependencies})

# Keep internal shared-library dependencies explicit. The Jetson toolchain
# resolves them while each shared object is linked: `vio` calls methods from
# `lio`, while `laser_mapping` calls all four processing libraries directly.
target_link_libraries(vio lio rgb_projection ${COMMON_DEPENDENCIES})
target_link_libraries(laser_mapping
  SQLite::SQLite3
  vio
  lio
  pre
  imu_proc
  ${nav_typesupport}
  vikit_common::vikit_common
  vikit_ros::vikit_ros
  ${COMMON_DEPENDENCIES}
)
target_link_libraries(lio utils ${COMMON_DEPENDENCIES})
target_link_libraries(pre ${COMMON_DEPENDENCIES})
target_link_libraries(imu_proc ${COMMON_DEPENDENCIES})

# Add the main executable
add_executable(fastlivo_mapping src/main.cpp)
add_executable(recover_map src/recover_map.cpp)
target_link_libraries(recover_map SQLite::SQLite3)

ament_target_dependencies(fastlivo_mapping ${dependencies})

# Link libraries to the executable
target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Link mimalloc if found
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

# Install the executable
install(TARGETS
  fastlivo_mapping
  recover_map
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS laser_mapping vio lio pre imu_proc utils rgb_projection
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
install(FILES runtime_policy.json DESTINATION share/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
  Eigen3
  PCL
  OpenCV
  Sophus
)

if(BUILD_TESTING)
  add_executable(adaptive_load_regression test/test_adaptive_load.cpp)
  add_test(NAME adaptive_load_regression COMMAND adaptive_load_regression)
  add_executable(patch_similarity_regression test/test_patch_similarity.cpp)
  add_test(NAME patch_similarity_regression COMMAND patch_similarity_regression)
  add_executable(patch_jacobian_regression test/test_patch_jacobian.cpp)
  add_test(NAME patch_jacobian_regression COMMAND patch_jacobian_regression)
  add_executable(patch_samples_regression test/test_patch_samples.cpp)
  target_link_libraries(patch_samples_regression OpenMP::OpenMP_CXX)
  add_test(NAME patch_samples_regression COMMAND patch_samples_regression)
  add_executable(voxel_index_regression test/test_voxel_index.cpp)
  add_test(NAME voxel_index_regression COMMAND voxel_index_regression)
  add_executable(measurement_variance_regression test/test_measurement_variance.cpp)
  add_test(NAME measurement_variance_regression COMMAND measurement_variance_regression)
  add_executable(lidar_point_covariance_regression test/test_lidar_point_covariance.cpp)
  add_test(NAME lidar_point_covariance_regression COMMAND lidar_point_covariance_regression)
  add_executable(plane_covariance_regression test/test_plane_covariance.cpp)
  add_test(NAME plane_covariance_regression COMMAND plane_covariance_regression)
  enable_testing()
  find_package(Threads REQUIRED)
  add_executable(latest_output_worker_regression test/test_latest_output_worker.cpp)
  target_link_libraries(latest_output_worker_regression Threads::Threads)
  add_test(NAME latest_output_worker_regression COMMAND latest_output_worker_regression)
  add_executable(async_path_preview_regression test/test_async_path_preview.cpp)
  ament_target_dependencies(async_path_preview_regression geometry_msgs nav_msgs)
  target_link_libraries(async_path_preview_regression Threads::Threads)
  add_test(NAME async_path_preview_regression COMMAND async_path_preview_regression)
  add_executable(parallelism_regression test/test_parallelism.cpp)
  target_link_libraries(parallelism_regression Threads::Threads OpenMP::OpenMP_CXX)
  add_test(NAME parallelism_regression COMMAND parallelism_regression)
  add_executable(runtime_log_path_regression test/test_runtime_log_path.cpp)
  target_link_libraries(runtime_log_path_regression Threads::Threads)
  add_test(NAME runtime_log_path_regression COMMAND runtime_log_path_regression)
  add_executable(lidar_slice_regression test/test_lidar_slice.cpp)
  add_test(NAME lidar_slice_regression COMMAND lidar_slice_regression)
  add_executable(lidar_slice_end_cache_regression test/test_lidar_slice_end_cache.cpp)
  add_test(NAME lidar_slice_end_cache_regression COMMAND lidar_slice_end_cache_regression)
  add_executable(sensor_receiver_regression test/test_sensor_receiver.cpp)
  ament_target_dependencies(sensor_receiver_regression rclcpp sensor_msgs)
  target_link_libraries(sensor_receiver_regression Threads::Threads)
  add_test(NAME sensor_receiver_regression COMMAND sensor_receiver_regression)
  find_package(rosgraph_msgs REQUIRED)
  add_executable(sensor_transport_regression test/test_sensor_transport.cpp)
  ament_target_dependencies(sensor_transport_regression rclcpp sensor_msgs rosgraph_msgs)
  target_link_libraries(sensor_transport_regression Threads::Threads)
  add_test(NAME sensor_transport_regression COMMAND sensor_transport_regression)
  set_tests_properties(sensor_transport_regression PROPERTIES TIMEOUT 25
    ENVIRONMENT "ROS_DOMAIN_ID=228;ROS_LOCALHOST_ONLY=1")
  add_executable(cloud_publication_regression test/test_cloud_publication.cpp)
  ament_target_dependencies(cloud_publication_regression rclcpp sensor_msgs)
  target_link_libraries(cloud_publication_regression Threads::Threads)
  add_test(NAME cloud_publication_regression COMMAND cloud_publication_regression)
  add_executable(sensor_inbox_regression test/test_sensor_inbox.cpp)
  target_link_libraries(sensor_inbox_regression Threads::Threads)
  add_test(NAME sensor_inbox_regression COMMAND sensor_inbox_regression)
  add_executable(async_state_log_regression test/test_async_state_log.cpp)
  target_link_libraries(async_state_log_regression Threads::Threads)
  add_test(NAME async_state_log_regression COMMAND async_state_log_regression)
  add_executable(map_preview_regression test/test_map_preview.cpp)
  add_test(NAME map_preview_regression COMMAND map_preview_regression)
  add_executable(navigation_map_regression test/test_navigation_map.cpp)
  ament_target_dependencies(navigation_map_regression ${dependencies})
  target_link_libraries(navigation_map_regression laser_mapping ${nav_typesupport} ${PCL_LIBRARIES})
  add_test(NAME navigation_map_regression COMMAND navigation_map_regression)
  add_executable(output_isolation_regression test/test_output_isolation.cpp)
  ament_target_dependencies(output_isolation_regression ${dependencies})
  target_link_libraries(output_isolation_regression laser_mapping ${nav_typesupport} ${PCL_LIBRARIES})
  add_test(NAME output_isolation_regression COMMAND output_isolation_regression)
  add_executable(checkpoint_burst_regression test/test_checkpoint_burst.cpp)
  ament_target_dependencies(checkpoint_burst_regression ${dependencies})
  target_link_libraries(checkpoint_burst_regression laser_mapping ${nav_typesupport} ${PCL_LIBRARIES})
  add_test(NAME checkpoint_burst_regression COMMAND checkpoint_burst_regression)
  add_executable(save_isolation_regression test/test_save_isolation.cpp)
  ament_target_dependencies(save_isolation_regression ${dependencies})
  target_link_libraries(save_isolation_regression laser_mapping ${nav_typesupport} ${PCL_LIBRARIES})
  add_test(NAME save_isolation_regression COMMAND save_isolation_regression)
  set_tests_properties(save_isolation_regression PROPERTIES TIMEOUT 35
    ENVIRONMENT "ROS_DOMAIN_ID=227;ROS_LOCALHOST_ONLY=1")
  add_executable(map_spool_regression test/test_map_spool.cpp)
  target_link_libraries(map_spool_regression SQLite::SQLite3)
  add_test(NAME map_spool_regression COMMAND map_spool_regression)
  add_executable(stop_signal_regression test/test_stop_signal.cpp)
  add_test(NAME stop_signal_regression COMMAND stop_signal_regression)
  add_executable(point_time_regression test/test_point_times.cpp)
  add_executable(woosh_cloud_regression test/test_woosh_cloud.cpp)
  ament_target_dependencies(woosh_cloud_regression ${dependencies})
  target_link_libraries(woosh_cloud_regression pre ${PCL_LIBRARIES} ${COMMON_DEPENDENCIES})
  add_test(NAME woosh_cloud_regression COMMAND woosh_cloud_regression)
  add_executable(parallel_residual_regression test/test_parallel_residual.cpp)
  ament_target_dependencies(parallel_residual_regression ${dependencies})
  target_link_libraries(parallel_residual_regression lio ${PCL_LIBRARIES} ${COMMON_DEPENDENCIES})
  add_test(NAME parallel_residual_regression COMMAND parallel_residual_regression)
  add_executable(parallel_voxel_update_regression test/test_parallel_voxel_update.cpp)
  ament_target_dependencies(parallel_voxel_update_regression ${dependencies})
  target_link_libraries(parallel_voxel_update_regression lio ${PCL_LIBRARIES} ${COMMON_DEPENDENCIES})
  foreach(team_size 1 2 4)
    add_test(NAME parallel_voxel_update_${team_size} COMMAND parallel_voxel_update_regression ${team_size})
  endforeach()
  add_executable(parallel_estimation_regression test/test_parallel_estimation.cpp)
  ament_target_dependencies(parallel_estimation_regression ${dependencies})
  target_link_libraries(parallel_estimation_regression lio ${PCL_LIBRARIES} ${COMMON_DEPENDENCIES})
  add_test(NAME parallel_estimation_regression COMMAND parallel_estimation_regression)
  add_executable(query_budget_regression test/test_query_budget.cpp)
  ament_target_dependencies(query_budget_regression ${dependencies})
  target_link_libraries(query_budget_regression lio ${PCL_LIBRARIES} ${COMMON_DEPENDENCIES})
  add_test(NAME query_budget_regression COMMAND query_budget_regression)
  ament_target_dependencies(point_time_regression ${dependencies})
  target_link_libraries(point_time_regression pre ${PCL_LIBRARIES} ${COMMON_DEPENDENCIES})
  add_test(NAME point_time_regression COMMAND point_time_regression)
  add_executable(imu_initialization_regression test/test_imu_initialization.cpp)
  ament_target_dependencies(imu_initialization_regression ${dependencies})
  target_link_libraries(imu_initialization_regression imu_proc ${PCL_LIBRARIES} ${COMMON_DEPENDENCIES})
  add_test(NAME imu_initialization_regression COMMAND imu_initialization_regression)
  add_executable(imu_deskew_regression test/test_imu_deskew.cpp)
  ament_target_dependencies(imu_deskew_regression ${dependencies})
  target_link_libraries(imu_deskew_regression imu_proc ${PCL_LIBRARIES} ${COMMON_DEPENDENCIES})
  add_test(NAME imu_deskew_regression COMMAND imu_deskew_regression)
  add_executable(memory_regression test/test_memory.cpp)
  ament_target_dependencies(memory_regression ${dependencies})
  target_link_libraries(memory_regression vio pre ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}
    vikit_common::vikit_common vikit_ros::vikit_ros ${COMMON_DEPENDENCIES})
  option(FAST_LIVO_MEMORY_SANITIZER "Check allocation ownership in memory_regression" OFF)
  if(FAST_LIVO_MEMORY_SANITIZER)
    # Match Eigen allocation ABI in the non-instrumented PCL/ROS libraries.
    # ASan otherwise changes Eigen's malloc detection only for this executable.
    target_compile_definitions(memory_regression PRIVATE EIGEN_MALLOC_ALREADY_ALIGNED=1)
    target_compile_options(memory_regression PRIVATE -fsanitize=address -fno-omit-frame-pointer)
    target_link_options(memory_regression PRIVATE -fsanitize=address -no-pie)
  endif()
  add_test(NAME memory_regression COMMAND memory_regression)
  add_executable(rgb_window_regression test/test_rgb_window.cpp)
  ament_target_dependencies(rgb_window_regression ${dependencies})
  target_link_libraries(rgb_window_regression laser_mapping ${nav_typesupport}
    vikit_common::vikit_common vikit_ros::vikit_ros ${COMMON_DEPENDENCIES})
  add_test(NAME rgb_window_regression COMMAND rgb_window_regression)
  add_executable(rgb_projection_snapshot_regression test/test_rgb_projection_snapshot.cpp)
  ament_target_dependencies(rgb_projection_snapshot_regression ${dependencies})
  target_link_libraries(rgb_projection_snapshot_regression vio
    vikit_common::vikit_common vikit_ros::vikit_ros ${COMMON_DEPENDENCIES})
  add_test(NAME rgb_projection_snapshot_regression COMMAND rgb_projection_snapshot_regression)
  add_executable(async_rgb_preview_regression test/test_async_rgb_preview.cpp)
  ament_target_dependencies(async_rgb_preview_regression ${dependencies})
  target_link_libraries(async_rgb_preview_regression Threads::Threads rgb_projection
    vikit_common::vikit_common ${OpenCV_LIBRARIES})
  add_test(NAME async_rgb_preview_regression COMMAND async_rgb_preview_regression)
  add_executable(async_rgb_mapper_regression test/test_async_rgb_mapper.cpp)
  ament_target_dependencies(async_rgb_mapper_regression ${dependencies})
  target_link_libraries(async_rgb_mapper_regression laser_mapping ${nav_typesupport}
    vikit_common::vikit_common vikit_ros::vikit_ros ${COMMON_DEPENDENCIES})
  add_test(NAME async_rgb_mapper_regression COMMAND async_rgb_mapper_regression)
  add_executable(camera_gap_regression test/test_camera_gap.cpp)
  ament_target_dependencies(camera_gap_regression ${dependencies})
  target_link_libraries(camera_gap_regression laser_mapping ${nav_typesupport}
    vikit_common::vikit_common vikit_ros::vikit_ros ${COMMON_DEPENDENCIES})
  add_test(NAME camera_gap_regression COMMAND camera_gap_regression)
  add_executable(tracking_faults_regression test/test_tracking_faults.cpp)
  add_executable(pose_output_bounds_regression test/test_pose_output_bounds.cpp)
  ament_target_dependencies(pose_output_bounds_regression nav_msgs)
  target_link_libraries(pose_output_bounds_regression Threads::Threads)
  add_test(NAME pose_output_bounds COMMAND pose_output_bounds_regression)
  set_tests_properties(pose_output_bounds PROPERTIES TIMEOUT 10)
  ament_target_dependencies(tracking_faults_regression ${dependencies})
  target_link_libraries(tracking_faults_regression laser_mapping ${nav_typesupport}
    vikit_common::vikit_common vikit_ros::vikit_ros ${COMMON_DEPENDENCIES})
  foreach(fault pose gap transform)
    add_test(NAME tracking_fault_${fault} COMMAND tracking_faults_regression ${fault})
    set_tests_properties(tracking_fault_${fault} PROPERTIES TIMEOUT 35
      ENVIRONMENT "ROS_DOMAIN_ID=229;ROS_LOCALHOST_ONLY=1")
  endforeach()
endif()

ament_package()
