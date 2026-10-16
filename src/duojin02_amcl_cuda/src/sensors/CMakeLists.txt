add_library(duojin02_amcl_sensors_lib SHARED
  laser/laser.cpp
  laser/beam_model.cpp
  laser/likelihood_field_model.cpp
  laser/likelihood_field_model_prob.cpp
)
# map_update_cspace
target_link_libraries(duojin02_amcl_sensors_lib duojin02_amcl_pf_lib duojin02_amcl_map_lib)

install(TARGETS
  duojin02_amcl_sensors_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
target_sources(duojin02_amcl_sensors_lib PRIVATE laser/observation_backend.cpp)
if(DUOJIN_ENABLE_CUDA)
  target_sources(duojin02_amcl_sensors_lib PRIVATE laser/observation_cuda.cu)
  target_compile_definitions(duojin02_amcl_sensors_lib PRIVATE DUOJIN02_AMCL_HAS_CUDA)
  target_link_libraries(duojin02_amcl_sensors_lib CUDA::cudart_static)
  set_target_properties(duojin02_amcl_sensors_lib PROPERTIES CUDA_RUNTIME_LIBRARY Static CUDA_STANDARD 17
    CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
  target_compile_options(duojin02_amcl_sensors_lib PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
endif()
