cmake_minimum_required(VERSION 3.24)
project(duojin02_amcl_cuda)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(duojin02_scan_geometry REQUIRED)

nav2_package()
# nav2_package warning switches target host compilers only, not nvcc.
get_directory_property(_nav2_host_options COMPILE_OPTIONS)
set_property(DIRECTORY PROPERTY COMPILE_OPTIONS "")
foreach(_option IN LISTS _nav2_host_options)
  add_compile_options("$<$<COMPILE_LANGUAGE:C,CXX>:${_option}>")
endforeach()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_INSTALL_RPATH "$ORIGIN")
option(DUOJIN_ENABLE_CUDA "Build CUDA AMCL observation backend" ON)
option(DUOJIN_CUDA_REQUIRED "Fail if requested CUDA compiler unavailable" OFF)
if(DUOJIN_ENABLE_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  if(NOT CMAKE_CUDA_COMPILER)
    if(DUOJIN_CUDA_REQUIRED)
      message(FATAL_ERROR "CUDA requested but nvcc not found")
    endif()
    set(DUOJIN_ENABLE_CUDA OFF)
  endif()
endif()
if(DUOJIN_ENABLE_CUDA)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES native)
  endif()
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
endif()

include_directories(
  include
)

include(CheckSymbolExists)
check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)

add_subdirectory(src/pf)
add_subdirectory(src/map)
add_subdirectory(src/motion_model)
add_subdirectory(src/sensors)

set(executable_name amcl)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name duojin02_amcl_core)

add_library(${library_name} SHARED
  src/amcl_node.cpp
)

set_target_properties(${executable_name} PROPERTIES INSTALL_RPATH "$ORIGIN/..")

target_include_directories(${library_name} PRIVATE src/include)

if(HAVE_DRAND48)
  target_compile_definitions(${library_name} PRIVATE "HAVE_DRAND48")
endif()

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  message_filters
  tf2_geometry_msgs
  geometry_msgs
  diagnostic_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf2_ros
  tf2
  nav2_util
  nav2_msgs
  pluginlib
  duojin02_scan_geometry
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name}
  ${library_name}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name}
  duojin02_amcl_map_lib duojin02_amcl_pf_lib duojin02_amcl_sensors_lib
)

rclcpp_components_register_nodes(${library_name} "nav2_amcl::AmclNode")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES UPSTREAM.json README.md LICENSE
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test/cuda)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name} duojin02_amcl_pf_lib duojin02_amcl_sensors_lib duojin02_amcl_motions_lib duojin02_amcl_map_lib)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(duojin02_amcl_cuda plugins.xml)
ament_package()
