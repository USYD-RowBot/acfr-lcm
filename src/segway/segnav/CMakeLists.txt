# SOURCE FILES
set (SRC
  main.c              
  navigator.c         navigator.h
  lcm_callbacks.c     lcm_callbacks.h
)

# EXTERNAL DEPS
perlsx_glib ()
perlsx_m ()

# BUILD EXE
perls_add_executable (segnav ${SRC})
perls_target_link_libraries (segnav perls-lcmtypes perls-common perls-math perls-est
  ${PERLSX_GLIB}
  ${PERLSX_M}
  )
