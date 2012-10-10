# EXTERNAL DEPS
perlsx_boost ()
perlsx_goby ()

# generate protobuf messages
set (PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/protodefs")
add_subdirectory (protodefs)

perls_add_executable (acomms main.cpp perls_acomms.h perls_acomms.cpp 
    perls_resource_manager.h perls_resource_manager.cpp) 
perls_target_link_libraries (acomms perls-lcmtypes perls-common perls-protobuf
  ${PERLSX_GOBY}
  ${PERLSX_BOOST}
  )

perls_add_executable (test-dccl-size test_dccl.cpp perls_resource_manager.h perls_resource_manager.cpp) 
perls_target_link_libraries (test-dccl-size perls-lcmtypes perls-common perls-protobuf
  ${PERLSX_GOBY}
  ${PERLSX_BOOST}
  )

perls_add_executable (acomms-command command.cpp)
perls_target_link_libraries (acomms-command perls-lcmtypes perls-common
    ${PERLSX_BOOST}
    )
