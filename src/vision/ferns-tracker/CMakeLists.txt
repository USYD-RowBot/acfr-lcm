# EXTERNAL DEPS
perlsx_opencv ()

# BUILD EXE
file (GLOB FERNS_SRC "ferns/*.cc")
file (GLOB FERNS_INC "ferns/*.h")
perls_add_executable (ferns-tracker main.cpp ${FERNS_INC} ${FERNS_SRC})
perls_target_link_libraries (ferns-tracker perls-lcmtypes perls-common perls-math
  ${PERLSX_OPENCV}
  )
