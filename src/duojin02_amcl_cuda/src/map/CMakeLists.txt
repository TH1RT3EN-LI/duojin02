add_library(duojin02_amcl_map_lib SHARED
  map.c
  map_range.c
  map_draw.c
  map_cspace.cpp
)

install(TARGETS
  duojin02_amcl_map_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
