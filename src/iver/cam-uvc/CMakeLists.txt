#perlsx_glib ()
#add_executable (cam-uvc main.c)
#target_link_libraries (cam-uvc perls-lcmtypes perls-common)

add_executable (cam-uvc cam-uvc.c)
target_link_libraries (cam-uvc perls-lcmtypes acfr-common)



