# BUILD EXE
perls_add_executable (leak main.c)
perls_target_link_libraries (leak perls-lcmtypes perls-common perls-math ${PERLSX_PTHREAD})

# build the kernel module


# Build the module
EXEC_PROGRAM(uname ARGS -r OUTPUT_VARIABLE KERNEL_VERSION)
Set( MODULE_SOURCE_DIR  ${CMAKE_CURRENT_BINARY_DIR} )

Set( KBUILD_CMD ${CMAKE_MAKE_PROGRAM}
                 -C /lib/modules/${KERNEL_VERSION}/build
                 M=${MODULE_SOURCE_DIR}
                 modules )
		 
ADD_CUSTOM_COMMAND(OUTPUT leak.ko
                   COMMAND ${KBUILD_CMD}
                   COMMENT "Building leak.ko"
		   DEPENDS leak.c
                  )

ADD_CUSTOM_TARGET(kleak ALL DEPENDS leak.ko)
