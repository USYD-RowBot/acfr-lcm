# SOURCE FILES
set (SRC
  main.c
  lcmlog_export.c  lcmlog_export.h
  bot_core.c       bot_core.h
  hauv.c           hauv.h
  perllcm.c        perllcm.h
  senlcm.c         senlcm.h
  se.c             se.h
)

# EXTERNAL DEPS
perlsx_glib ()

# BUILD EXE
perls_add_executable (lcmlog-export ${SRC})
perls_target_link_libraries (lcmlog-export perls-lcmtypes perls-common
  ${PERLSX_GLIB}
  )
