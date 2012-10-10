# SOURCE FILES
set (SRC
  main.c              
  navigator.c         navigator.h
  lcm_callbacks.c     lcm_callbacks.h
)

# EXTERNAL DEPS
perlsx_bot2 ()
perlsx_glib ()
perlsx_m ()


# BUILD EXE
perls_add_executable (navigator ${SRC})
perls_target_link_libraries (navigator perls-lcmtypes perls-common perls-math perls-est
  ${PERLSX_BOT2}
  ${PERLSX_GLIB}
  ${PERLSX_M}
  )
