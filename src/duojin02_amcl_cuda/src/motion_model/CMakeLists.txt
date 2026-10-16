add_library(duojin02_amcl_motions_lib SHARED
  omni_motion_model.cpp
  differential_motion_model.cpp
)
target_link_libraries(duojin02_amcl_motions_lib duojin02_amcl_pf_lib)
ament_target_dependencies(duojin02_amcl_motions_lib
  pluginlib
  nav2_util
)

install(TARGETS
  duojin02_amcl_motions_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
