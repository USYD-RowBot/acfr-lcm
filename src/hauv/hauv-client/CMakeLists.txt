# SOURCE FILES
set (SRC
  main.cpp
  HauvClient.cpp            HauvClient.h
)

# EXTERNAL DEPS
perlsx_eigen3 ()

# BUILD EXE
perls_add_executable (hauv-client ${SRC})
perls_target_link_libraries (hauv-client perls-common perls-lcmtypes perls-math
  perls-hauv
  ${PERLSX_EIGEN3}
  )
