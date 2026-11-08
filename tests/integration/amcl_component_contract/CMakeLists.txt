cmake_minimum_required(VERSION 3.24)
project(amcl_component_contract LANGUAGES C CXX CUDA)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_ARCHITECTURES 120)
find_package(CUDAToolkit REQUIRED)
get_filename_component(REPO "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(AMCL "${REPO}/src/duojin02_amcl_cuda")
include_directories("${AMCL}/include" "${REPO}/src/duojin02_scan_geometry/include" "${REPO}/tests/integration")
add_library(contract_pf SHARED "${AMCL}/src/pf/pf.c" "${AMCL}/src/pf/pf_kdtree.c"
  "${AMCL}/src/pf/pf_pdf.c" "${AMCL}/src/pf/pf_vector.c" "${AMCL}/src/pf/eig3.c")
target_compile_definitions(contract_pf PRIVATE HAVE_DRAND48)
target_link_libraries(contract_pf m)
add_library(contract_map SHARED "${AMCL}/src/map/map.c" "${AMCL}/src/map/map_range.c"
  "${AMCL}/src/map/map_cspace.cpp")
add_library(contract_sensor SHARED "${AMCL}/src/sensors/laser/laser.cpp"
  "${AMCL}/src/sensors/laser/beam_model.cpp" "${AMCL}/src/sensors/laser/likelihood_field_model.cpp"
  "${AMCL}/src/sensors/laser/likelihood_field_model_prob.cpp"
  "${AMCL}/src/sensors/laser/observation_backend.cpp" "${AMCL}/src/sensors/laser/observation_cuda.cu")
target_compile_definitions(contract_sensor PRIVATE DUOJIN02_AMCL_HAS_CUDA)
target_compile_options(contract_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
target_link_libraries(contract_sensor contract_pf contract_map CUDA::cudart_static)
add_executable(amcl_observation_contract "${AMCL}/test/cuda/observation_test.cpp")
target_compile_definitions(amcl_observation_contract PRIVATE
  DUOJIN02_RECORDED_SCAN_FIXTURE="${REPO}/tests/integration/fixtures/lilocbench_geometry_scan.txt")
target_link_libraries(amcl_observation_contract contract_sensor)
add_executable(amcl_pf_contract "${AMCL}/test/cuda/pf_numerics_test.cpp")
target_link_libraries(amcl_pf_contract contract_pf)
enable_testing()
add_test(NAME actual_cpu_score_reference_contract COMMAND amcl_observation_contract cpu)
add_test(NAME actual_cuda_score_reference_contract COMMAND amcl_observation_contract cuda)
add_test(NAME particle_filter_contract COMMAND amcl_pf_contract)
