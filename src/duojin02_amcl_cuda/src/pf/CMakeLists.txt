if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-gnu-folding-constant)
endif()

add_library(duojin02_amcl_pf_lib SHARED
  pf.c
  pf_kdtree.c
  pf_pdf.c
  pf_vector.c
  eig3.c
  pf_draw.c
)

target_include_directories(duojin02_amcl_pf_lib PRIVATE ../include)
if(HAVE_DRAND48)
  target_compile_definitions(duojin02_amcl_pf_lib PRIVATE "HAVE_DRAND48")
endif()
target_link_libraries(duojin02_amcl_pf_lib m)

install(TARGETS
  duojin02_amcl_pf_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
