cmake_minimum_required(VERSION 3.8)
project(slam_input_probe)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization thread)
find_package(TBB REQUIRED)
add_executable(scan_gate_probe scan_gate_probe.cpp)
ament_target_dependencies(scan_gate_probe rclcpp slam_toolbox)
find_library(
  TOOLBOX_COMMON_LIBRARY
  NAMES toolbox_common
  HINTS ${slam_toolbox_DIR}/../../../lib
  NO_DEFAULT_PATH REQUIRED)
find_library(
  KARTO_LIBRARY
  NAMES kartoSlamToolbox
  HINTS ${slam_toolbox_DIR}/../../../lib
  NO_DEFAULT_PATH REQUIRED)
target_link_libraries(scan_gate_probe ${TOOLBOX_COMMON_LIBRARY} ${KARTO_LIBRARY}
                      Boost::serialization Boost::thread TBB::tbb)
find_package(Ceres REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
get_filename_component(HARDWARE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(PREPARED_SLAM_SOURCE
    ""
    CACHE PATH "Existing verified SLAM test source; empty selects cache or Docker testing source")
set(SLAM_SOURCE_ARGUMENTS --root "${HARDWARE_ROOT}")
if(PREPARED_SLAM_SOURCE)
  list(APPEND SLAM_SOURCE_ARGUMENTS --source "${PREPARED_SLAM_SOURCE}")
endif()
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/slam_test_source.py"
          ${SLAM_SOURCE_ARGUMENTS}
  RESULT_VARIABLE SLAM_SOURCE_RESULT
  OUTPUT_VARIABLE VERIFIED_SLAM_SOURCE
  ERROR_VARIABLE SLAM_SOURCE_ERROR
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT SLAM_SOURCE_RESULT STREQUAL "0")
  message(FATAL_ERROR "${SLAM_SOURCE_ERROR}")
endif()
set(PREPARED_SLAM_SOURCE "${VERIFIED_SLAM_SOURCE}")
add_executable(ceres_anchor_probe ceres_anchor_probe.cpp
                                  "${PREPARED_SLAM_SOURCE}/solvers/ceres_solver.cpp")
target_include_directories(
  ceres_anchor_probe BEFORE
  PRIVATE "${PREPARED_SLAM_SOURCE}" "${PREPARED_SLAM_SOURCE}/include"
          "${PREPARED_SLAM_SOURCE}/lib/karto_sdk/include" ${slam_toolbox_INCLUDE_DIRS})
ament_target_dependencies(
  ceres_anchor_probe
  rclcpp
  slam_toolbox
  std_srvs
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs)
target_link_libraries(ceres_anchor_probe ${KARTO_LIBRARY} Ceres::ceres Boost::serialization
                      Boost::thread TBB::tbb)
add_executable(map_saver_completion_probe map_saver_completion_probe.cpp
                                          "${PREPARED_SLAM_SOURCE}/src/map_saver.cpp")
target_include_directories(map_saver_completion_probe BEFORE
                           PRIVATE "${PREPARED_SLAM_SOURCE}/include")
ament_target_dependencies(
  map_saver_completion_probe
  rclcpp
  slam_toolbox
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs)
