# BUILD EXE
perls_add_executable (vimba-cam vimba.c)
perls_target_link_libraries (vimba-cam perls-lcmtypes  VimbaC tiff acfr-common
  )
