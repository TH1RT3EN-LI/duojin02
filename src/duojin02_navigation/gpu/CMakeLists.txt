cmake_minimum_required(VERSION 3.18)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(duojin02_geometry LANGUAGES CXX)
  include(CTest)
endif()

option(DUOJIN_ENABLE_CUDA "Build GPU geometry when a compatible CUDA compiler is available" ON)
option(DUOJIN_CUDA_REQUIRED "Fail configuration or GPU checks if CUDA is unavailable" OFF)
if(DUOJIN_CUDA_REQUIRED AND NOT DUOJIN_ENABLE_CUDA)
  message(FATAL_ERROR "DUOJIN_CUDA_REQUIRED=ON requires DUOJIN_ENABLE_CUDA=ON")
endif()

add_library(duojin02_gpu_geometry SHARED geometry.cpp)
target_include_directories(
  duojin02_gpu_geometry PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
                               $<INSTALL_INTERFACE:include>)
target_compile_features(duojin02_gpu_geometry PUBLIC cxx_std_17)
set(DUOJIN_CUDA_BUILT OFF)
if(DUOJIN_ENABLE_CUDA)
  # Choose the installed desktop GPU, rather than silently targeting an Orin.
  # native requires CMake 3.24; older CMake can target this local RTX 50 series explicitly.
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
      set(CMAKE_CUDA_ARCHITECTURES native CACHE STRING "CUDA GPU architectures")
    else()
      set(CMAKE_CUDA_ARCHITECTURES 120 CACHE STRING "CUDA GPU architectures")
    endif()
  endif()
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)
    target_sources(duojin02_gpu_geometry PRIVATE geometry.cu)
    target_compile_definitions(duojin02_gpu_geometry PRIVATE DUOJIN_HAS_CUDA=1)
    target_compile_options(
      duojin02_gpu_geometry
      PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--ftz=false> $<$<COMPILE_LANGUAGE:CUDA>:--prec-div=true>
              $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
    set_target_properties(duojin02_gpu_geometry PROPERTIES
      CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON
      CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
    target_link_libraries(duojin02_gpu_geometry PRIVATE CUDA::cudart_static)
    set(DUOJIN_CUDA_BUILT ON)
    message(STATUS "duojin02 GPU geometry: CUDA enabled, architectures=${CMAKE_CUDA_ARCHITECTURES}")
  elseif(DUOJIN_CUDA_REQUIRED)
    message(FATAL_ERROR "DUOJIN_CUDA_REQUIRED=ON but no compatible CUDA compiler was found")
  else()
    message(STATUS "duojin02 GPU geometry: no CUDA compiler; building complete CPU support")
  endif()
else()
  message(STATUS "duojin02 GPU geometry: DUOJIN_ENABLE_CUDA=OFF; building CPU support")
endif()
install(TARGETS duojin02_gpu_geometry LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  add_executable(gpu_geometry_checks geometry_checks.cpp)
  target_link_libraries(gpu_geometry_checks duojin02_gpu_geometry)
  target_compile_options(gpu_geometry_checks PRIVATE -UNDEBUG)
  add_test(NAME gpu_geometry_checks COMMAND gpu_geometry_checks)
  add_test(NAME gpu_geometry_no_device_checks COMMAND gpu_geometry_checks --no-device)
  set_tests_properties(gpu_geometry_no_device_checks PROPERTIES ENVIRONMENT "CUDA_VISIBLE_DEVICES=")
  if(DUOJIN_CUDA_BUILT)
    add_test(NAME gpu_geometry_cuda_checks COMMAND gpu_geometry_checks --require-cuda)
    if(NOT DUOJIN_CUDA_REQUIRED)
      set_tests_properties(gpu_geometry_cuda_checks PROPERTIES SKIP_RETURN_CODE 77)
    endif()
    add_executable(gpu_geometry_fault_checks geometry_fault_checks.cu)
    target_link_libraries(gpu_geometry_fault_checks PRIVATE duojin02_gpu_geometry
                                                            CUDA::cudart_static)
    target_compile_options(gpu_geometry_fault_checks PRIVATE -UNDEBUG)
    set_target_properties(gpu_geometry_fault_checks PROPERTIES CUDA_STANDARD 17
                                                               CUDA_STANDARD_REQUIRED ON)
    add_test(NAME gpu_geometry_fault_checks COMMAND gpu_geometry_fault_checks)
    set_tests_properties(gpu_geometry_fault_checks PROPERTIES TIMEOUT 30)
    if(NOT DUOJIN_CUDA_REQUIRED)
      set_tests_properties(gpu_geometry_fault_checks PROPERTIES SKIP_RETURN_CODE 77)
    endif()
  endif()
  add_executable(
    gpu_geometry_benchmark EXCLUDE_FROM_ALL
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/performance/gpu_geometry_benchmark.cpp")
  target_link_libraries(gpu_geometry_benchmark duojin02_gpu_geometry)
endif()
