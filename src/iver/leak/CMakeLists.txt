# BUILD EXE
perls_add_executable (leak main.c)
perls_target_link_libraries (leak perls-lcmtypes perls-common perls-math ${PERLSX_PTHREAD})

# build the kernel module


# Build the module

#ADD_CUSTOM_COMMAND(OUTPUT leak.ko
#                   COMMAND make -f module.make
                   #DEPENDS leak.c Kbuild
#                   COMMENT "Building leak.ko"
#                  )

#ADD_CUSTOM_TARGET(kleak ALL DEPENDS leak.ko)
