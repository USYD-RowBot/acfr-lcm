# EXTERNAL DEPS
perlsx_glib ()
perlsx_boost ()
perlsx_goby ()

# generate protobuf messages
set (PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/protodefs")
add_subdirectory (protodefs)

# perls_add_executable (goby-acomms main.cpp perls_acomms.h goby_acomms.h lcm_listeners.cpp lcm_listeners.h)
# perls_target_link_libraries (goby-acomms perls-lcmtypes perls-common perls-protobuf
#   ${PERLSX_GOBY}
#   ${PERLSX_BOOST}
#   )

#perls_add_executable (capture-client capture_client.cpp lcm_listeners.cpp lcm_listeners.h)
#perls_target_link_libraries (capture-client perls-lcmtypes perls-common perls-protobuf
#  ${PERLSX_BOOST}
#  ${PERLSX_GLIB}
#  ${PERLSX_GOBY}
#  )
