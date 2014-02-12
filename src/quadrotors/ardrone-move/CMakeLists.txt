# EXTERNAL DEPS
perlsx_m ()

# BUILD EXE
perls_add_executable (ardrone-move-xbox ardrone_move_xbox.c)
perls_target_link_libraries (ardrone-move-xbox perls-lcmtypes perls-common
  ${PERLSX_M}
  )

# BUILD EXE
perls_add_executable (ardrone-move-pid ardrone_move_pid.c)
perls_target_link_libraries (ardrone-move-pid perls-lcmtypes perls-common perls-math
  ${PERLSX_M}
  )
