# SOURCE FILES
set (SRC
  main.cpp
  SonarClient.cpp
  didson.cpp
  ndt.cpp
  scanbuilder.cpp
)

# EXTERNAL DEPS
perlsx_bot2 ()
perlsx_lcm ()
perlsx_eigen3 ()
perlsx_opencv ()

# BUILD EXE
perls_add_executable (sonar-client ${SRC})
perls_target_link_libraries (sonar-client
  perls-hauv
  perls-lcmtypes
  perls-common
  perls-math
  ${PERLSX_BOT2}
  ${PERLSX_LCM}
  ${PERLSX_EIGEN3}
  ${PERLSX_OPENCV}
  boost_thread-mt
  )
