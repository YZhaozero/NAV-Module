cmake_minimum_required(VERSION 3.8)
project(localizer)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OpenMP REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interface REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common
  io
  registration
  filters
  features
  sample_consensus
)
find_package(yaml-cpp REQUIRED)

# Find TEASER++
find_package(teaserpp REQUIRED)
message(STATUS "TEASER++ found: ${teaserpp_FOUND}")
find_package(GTSAM REQUIRED)
find_package(TBB REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${TBB_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include
)


set(SRC_LIST src/localizers/commons.cpp
  src/localizers/icp_localizer.cpp
  src/localizers/matcher.cc)

add_executable(teaser_solve_worker src/teaser_solve_worker.cpp)
target_link_libraries(teaser_solve_worker teaserpp::teaser_registration)
add_executable(localizer_node src/localizer_node.cpp ${SRC_LIST})
add_dependencies(localizer_node teaser_solve_worker)
ament_target_dependencies(localizer_node rclcpp rcl_interfaces std_msgs std_srvs
  sensor_msgs nav_msgs message_filters pcl_conversions tf2_ros geometry_msgs
  visualization_msgs interface)
target_link_libraries(localizer_node
  ${OpenCV_LIBS}
  OpenSSL::Crypto
  ${PCL_LIBRARIES}
  yaml-cpp
  teaserpp::teaser_registration
  teaserpp::teaser_io
  gtsam
  TBB::tbb
  OpenMP::OpenMP_CXX
)

add_executable(generate_sc_tool src/generate_sc_tool.cpp ${SRC_LIST})
target_link_libraries(generate_sc_tool
  ${OpenCV_LIBS}
  OpenSSL::Crypto
  ${PCL_LIBRARIES}
  yaml-cpp
  teaserpp::teaser_registration
  teaserpp::teaser_io
  gtsam
  TBB::tbb
  OpenMP::OpenMP_CXX
)

install(TARGETS localizer_node generate_sc_tool teaser_solve_worker
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(correlative_search_test test/correlative_search_test.cpp)
  target_link_libraries(correlative_search_test ${PCL_LIBRARIES} ${OpenCV_LIBS})
  target_compile_options(correlative_search_test PRIVATE -UNDEBUG)
  add_test(NAME correlative_search COMMAND correlative_search_test)
  add_executable(adaptive_correlative_simulation_test test/adaptive_correlative_simulation_test.cpp)
  target_link_libraries(adaptive_correlative_simulation_test ${PCL_LIBRARIES} ${OpenCV_LIBS})
  target_compile_options(adaptive_correlative_simulation_test PRIVATE -UNDEBUG)
  add_test(NAME adaptive_correlative_simulation COMMAND adaptive_correlative_simulation_test)
  add_executable(correlative_search_probe test/correlative_search_probe.cpp)
  target_link_libraries(correlative_search_probe ${PCL_LIBRARIES} ${OpenCV_LIBS})
  add_executable(adaptive_localizer_probe test/adaptive_localizer_probe.cpp ${SRC_LIST})
  target_link_libraries(adaptive_localizer_probe ${OpenCV_LIBS} OpenSSL::Crypto
    ${PCL_LIBRARIES} yaml-cpp teaserpp::teaser_registration teaserpp::teaser_io
    gtsam TBB::tbb OpenMP::OpenMP_CXX)
  add_executable(teaser_process_test test/teaser_process_test.cpp)
  target_link_libraries(teaser_process_test teaserpp::teaser_registration)
  target_compile_options(teaser_process_test PRIVATE -UNDEBUG)
  add_dependencies(teaser_process_test teaser_solve_worker)
  add_test(NAME teaser_process COMMAND teaser_process_test)
  add_executable(tracking_policy_test test/tracking_policy_test.cpp)
  target_compile_options(tracking_policy_test PRIVATE -UNDEBUG)
  add_test(NAME tracking_policy COMMAND tracking_policy_test)
  add_executable(initialization_window_test test/initialization_window_test.cpp)
  target_link_libraries(initialization_window_test ${PCL_LIBRARIES})
  target_compile_options(initialization_window_test PRIVATE -UNDEBUG)
  add_test(NAME initialization_window COMMAND initialization_window_test)
  add_executable(global_features_test test/global_features_test.cpp)
  target_link_libraries(global_features_test ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
  target_compile_options(global_features_test PRIVATE -UNDEBUG)
  add_test(NAME global_features COMMAND global_features_test)
  add_executable(cancellable_search_test test/cancellable_search_test.cpp)
  target_link_libraries(cancellable_search_test ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
  target_compile_options(cancellable_search_test PRIVATE -UNDEBUG)
  add_test(NAME cancellable_search COMMAND cancellable_search_test)
  add_executable(input_policy_test test/input_policy_test.cpp)
  ament_target_dependencies(input_policy_test sensor_msgs)
  target_compile_options(input_policy_test PRIVATE -UNDEBUG)
  add_test(NAME input_policy COMMAND input_policy_test)
  add_executable(scan_context_test test/scan_context_test.cpp)
  target_link_libraries(scan_context_test ${PCL_LIBRARIES})
  target_compile_options(scan_context_test PRIVATE -UNDEBUG)
  add_test(NAME scan_context COMMAND scan_context_test)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
