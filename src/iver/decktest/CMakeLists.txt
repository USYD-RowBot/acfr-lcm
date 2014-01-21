# EXTERNAL DEPS
perlsx_bot2 ()
perlsx_m ()

perls_add_executable (decktest main.c)
perls_target_link_libraries (decktest perls-lcmtypes perls-common perls-iver
  ${PERLSX_BOT2}
  ${PERLSX_M}
  )
