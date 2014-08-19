add_executable (acfr-nav acfrNav.cpp main.cpp magfield.c)
target_link_libraries (acfr-nav perls-lcmtypes perls-common seabedcommon seabedslam adt plankton flounder)

#add_executable (acfr-nav-post acfrNav.cpp main-post.cpp magfield.c)
#target_link_libraries (acfr-nav-post perls-lcmtypes perls-common seabedcommon seabedslam)

#add_executable (lcm2raw acfrNav.cpp main_l2r.cpp magfield.c)
#target_link_libraries (lcm2raw perls-lcmtypes perls-common seabedcommon seabedslam adt plankton flounder)

include_directories ( 
	/usr/local/include/libseabedcommon
	/usr/local/include/libadt
	/usr/local/include/libplankton
	/usr/local/include/libflounder
	/usr/local/include/libsnapper
	)				
link_directories (/usr/local/lib)


# Install the lcm2raw binary on 'make install'
INSTALL_TARGETS( /bin lcm2raw )

