cmake_minimum_required(VERSION 3.8)
project(lslidar_driver VERSION 5.1.3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

add_library(lslidar_driver src/lsiosr.cpp src/lslidar_x10_driver.cpp)
target_include_directories(
  lslidar_driver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include> ${PCL_INCLUDE_DIRS})
target_link_libraries(lslidar_driver ${PCL_LIBRARIES})
ament_target_dependencies(lslidar_driver rclcpp rcl_interfaces sensor_msgs std_msgs pcl_conversions)

add_executable(lslidar_driver_node src/lslidar_driver_node.cpp)
target_link_libraries(lslidar_driver_node lslidar_driver)

install(TARGETS lslidar_driver lslidar_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(lslidar_driver)
ament_export_dependencies(rclcpp rcl_interfaces sensor_msgs std_msgs pcl_conversions PCL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_x10_driver test/test_x10_driver.cpp TIMEOUT 30)
  target_link_libraries(test_x10_driver lslidar_driver)
  target_compile_definitions(
    test_x10_driver
    PRIVATE HARDWARE_LIDAR_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/duojin02_n10plus.yaml")
endif()

ament_package()
