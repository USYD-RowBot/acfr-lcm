# EXTERNAL DEPS
perlsx_m ()

# BUILD EXE
perls_add_executable (ardrone-cmd-xbox ardrone_cmd_xbox.c)
perls_target_link_libraries (ardrone-cmd-xbox perls-lcmtypes perls-common
  ${PERLSX_M}
  )

