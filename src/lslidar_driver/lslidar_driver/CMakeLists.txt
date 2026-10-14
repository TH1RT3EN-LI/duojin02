cmake_minimum_required(VERSION 3.8)
project(lslidar_driver VERSION 5.1.3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

add_library(lslidar_driver src/lsiosr.cpp src/lslidar_x10_driver.cpp)
target_include_directories(lslidar_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(lslidar_driver ${PCL_LIBRARIES})
ament_target_dependencies(lslidar_driver rclcpp sensor_msgs std_msgs pcl_conversions)

add_executable(lslidar_driver_node src/lslidar_driver_node.cpp)
target_link_libraries(lslidar_driver_node lslidar_driver)

install(TARGETS lslidar_driver lslidar_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_libraries(lslidar_driver)
ament_export_dependencies(rclcpp sensor_msgs std_msgs pcl_conversions PCL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  get_filename_component(DUOJIN_CHASSIS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
  set(DUOJIN_LIDAR_TEST_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/lidar-derived.yaml")
  add_custom_command(OUTPUT "${DUOJIN_LIDAR_TEST_CONFIG}"
    COMMAND ${Python3_EXECUTABLE} "${DUOJIN_CHASSIS_ROOT}/scripts/chassis_config.py" export
      --profile "${DUOJIN_CHASSIS_ROOT}/src/duojin01_bringup/config/chassis.yaml"
      --component lidar --output "${DUOJIN_LIDAR_TEST_CONFIG}"
    DEPENDS
      "${DUOJIN_CHASSIS_ROOT}/src/duojin01_bringup/config/chassis.yaml"
      "${DUOJIN_CHASSIS_ROOT}/src/duojin01_bringup/duojin01_bringup/chassis_profile.py"
      "${DUOJIN_CHASSIS_ROOT}/scripts/chassis_config.py"
    VERBATIM)
  add_custom_target(lidar_test_parameters DEPENDS "${DUOJIN_LIDAR_TEST_CONFIG}")
  ament_add_gtest(test_self_filter test/test_self_filter.cpp TIMEOUT 30)
  target_include_directories(test_self_filter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_x10_driver test/test_x10_driver.cpp TIMEOUT 30)
  target_link_libraries(test_x10_driver lslidar_driver)
  add_dependencies(test_x10_driver lidar_test_parameters)
  target_compile_definitions(test_x10_driver PRIVATE
    HARDWARE_LIDAR_CONFIG="${DUOJIN_LIDAR_TEST_CONFIG}")
  # Developer-only saved-byte replay; never installed as a hardware executable.
  add_executable(replay_real_lidar_crop ../../../tests/performance/replay_real_lidar_crop.cpp)
  target_link_libraries(replay_real_lidar_crop lslidar_driver)
endif()
ament_package()
