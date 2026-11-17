# Copyright(c) 2020 livoxtech limited.

cmake_minimum_required(VERSION 3.14)
project(livox_ros_driver2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Printf version info
include(cmake/version.cmake)
project(${PROJECT_NAME} VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
# Incremental device releases do not pass CMAKE_BUILD_TYPE. Leaving it empty
# compiles the per-point conversion and STL queues without optimization.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "${PROJECT_NAME} version: ${LIVOX_ROS_DRIVER2_VERSION}")

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate custom msg headers
set(LIVOX_INTERFACES livox_interfaces2)
rosidl_generate_interfaces(${LIVOX_INTERFACES}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)

## make sure the livox_lidar_sdk_shared library is installed
find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_shared.so /usr/local/lib REQUIRED)

##
find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR
  NAMES "livox_lidar_api.h" "livox_lidar_def.h"
  REQUIRED)

# MID-360 ROS 2 driver executable
ament_auto_add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/livox_ros_driver2.cpp
  src/lddc.cpp
  src/driver_node.cpp
  src/lds.cpp
  src/lds_lidar.cpp

  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp
  src/comm/packet_evidence.cpp

  src/parse_cfg_file/parse_livox_lidar_cfg.cpp

  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

# get include directories of custom msg headers
if($ENV{ROS_DISTRO} STREQUAL "foxy")
  set(LIVOX_INTERFACE_TARGET "${LIVOX_INTERFACES}__rosidl_typesupport_cpp")
  add_dependencies(${PROJECT_NAME}_node ${LIVOX_INTERFACES})
  get_target_property(LIVOX_INTERFACES_INCLUDE_DIRECTORIES ${LIVOX_INTERFACE_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
else()
  rosidl_get_typesupport_target(cpp_typesupport_target
  ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")
  target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")
endif()

# include file direcotry
target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
  ${LIVOX_INTERFACES_INCLUDE_DIRECTORIES}   # for custom msgs
  3rdparty
  src
)

# link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${LIVOX_LIDAR_SDK_LIBRARY}
  ${LIVOX_INTERFACE_TARGET}   # for custom msgs
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_device_clock test/test_device_clock.cpp)
  target_include_directories(test_device_clock PRIVATE src)
  add_test(NAME device_clock COMMAND test_device_clock)
  find_package(Threads REQUIRED)
  add_executable(test_lidar_queue test/test_lidar_queue.cpp src/comm/ldq.cpp)
  target_include_directories(test_lidar_queue PRIVATE src)
  target_link_libraries(test_lidar_queue Threads::Threads)
  add_test(NAME lidar_queue COMMAND test_lidar_queue)
  ament_auto_add_executable(test_preview_publication
    test/test_preview_publication.cpp src/lddc.cpp src/lds.cpp
    src/comm/comm.cpp src/comm/ldq.cpp src/comm/semaphore.cpp
    src/comm/lidar_imu_data_queue.cpp src/comm/cache_index.cpp)
  target_include_directories(test_preview_publication PRIVATE src ${LIVOX_LIDAR_SDK_INCLUDE_DIR})
  target_link_libraries(test_preview_publication "${cpp_typesupport_target}" Threads::Threads)
  add_test(NAME preview_publication COMMAND test_preview_publication)
  set_tests_properties(preview_publication PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=221;ROS_LOCALHOST_ONLY=1" TIMEOUT 15)
  add_test(NAME preview_publication_multi COMMAND test_preview_publication)
  set_tests_properties(preview_publication_multi PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=225;ROS_LOCALHOST_ONLY=1;LIVOX_TEST_MULTI=1" TIMEOUT 15)
  ament_auto_add_executable(test_woosh_multi_lidar
    test/test_woosh_multi_lidar.cpp src/lddc.cpp src/lds.cpp
    src/comm/comm.cpp src/comm/ldq.cpp src/comm/semaphore.cpp
    src/comm/lidar_imu_data_queue.cpp src/comm/cache_index.cpp
    src/comm/pub_handler.cpp src/comm/packet_evidence.cpp)
  target_include_directories(test_woosh_multi_lidar PRIVATE src ${LIVOX_LIDAR_SDK_INCLUDE_DIR})
  target_link_libraries(test_woosh_multi_lidar "${cpp_typesupport_target}" Threads::Threads)
  add_test(NAME woosh_multi_lidar COMMAND test_woosh_multi_lidar)
  set_tests_properties(woosh_multi_lidar PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=227;ROS_LOCALHOST_ONLY=1;NAV_EVIDENCE_DIR=/tmp/woosh-multi-lidar-test" TIMEOUT 15)
  ament_auto_add_executable(test_lidar_shutdown
    test/test_lidar_shutdown.cpp src/lddc.cpp src/lds.cpp
    src/comm/comm.cpp src/comm/ldq.cpp src/comm/semaphore.cpp
    src/comm/lidar_imu_data_queue.cpp src/comm/cache_index.cpp)
  target_include_directories(test_lidar_shutdown PRIVATE src ${LIVOX_LIDAR_SDK_INCLUDE_DIR})
  target_link_libraries(test_lidar_shutdown "${cpp_typesupport_target}" Threads::Threads)
  add_test(NAME lidar_shutdown COMMAND test_lidar_shutdown)
  set_tests_properties(lidar_shutdown PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=226;ROS_LOCALHOST_ONLY=1" TIMEOUT 10)
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
