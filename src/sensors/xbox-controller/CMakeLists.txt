# EXTERNAL DEPS
perlsx_m ()
perlsx_dbus ()

# BUILD EXE
perls_add_executable (xbox-controller main.c)
perls_target_link_libraries (xbox-controller perls-lcmtypes perls-common
  ${PERLSX_M}
  ${PERLSX_DBUS}
  )
