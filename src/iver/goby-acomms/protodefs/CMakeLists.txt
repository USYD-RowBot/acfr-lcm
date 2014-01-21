# FIND PROTOBUF EXE
find_program (PROTOC_EXE protoc DOC "Location of Protobuf protoc executable.")
if (PROTOC_EXE)
  mark_as_advanced (PROTOC_EXE)
else ()
  message (SEND_ERROR "protobuf protoc executable not found!; on ubuntu `sudo apt-get install protobuf-compiler python-protobuf`")
endif ()

# GENERATE PROTOBUF BINDINGS
set (GENERATED_PROTOBUF_HEADERS "")
set (GENERATED_PROTOBUF_SOURCES "")
set (GENERATED_PROTOBUF_PY "")

file (GLOB PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.proto)
foreach (PROTO_FILE ${PROTO_FILES})
  get_filename_component (PROTO_NAME ${PROTO_FILE} NAME_WE)
  set (PROTO_PATH "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}")
  
  add_custom_command (
    OUTPUT ${PROTO_PATH}.pb.cc ${PROTO_PATH}.pb.h ${PROTO_PATH}_pb2.py
    COMMAND ${PROTOC_EXE} ${PROTO_FILE} --python_out=${CMAKE_CURRENT_BINARY_DIR} --cpp_out=${CMAKE_CURRENT_BINARY_DIR} -I${CMAKE_CURRENT_BINARY_DIR} -I/usr/local/include/ -I/usr/include/ -I${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS ${PROTO_FILE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

  list (APPEND GENERATED_PROTOBUF_SOURCES ${PROTO_PATH}.pb.cc)
  list (APPEND GENERATED_PROTOBUF_HEADERS ${PROTO_PATH}.pb.h)
  list (APPEND GENERATED_PROTOBUF_PY ${PROTO_PATH}_pb2.py)
endforeach ()

# EXTERNAL DEPS
perlsx_goby ()
perlsx_protobuf ()

# BUILD LIB
perls_add_library (perls-protobuf ${GENERATED_PROTOBUF_HEADERS} ${GENERATED_PROTOBUF_SOURCES})
perls_target_link_libraries (perls-protobuf
  ${PERLSX_GOBY}
  ${PERLSX_PROTOBUF}
  )
set_target_properties (perls-protobuf PROPERTIES COMPILE_FLAGS -fPIC)

# INSTALL LCM-GENERATED HEADERS
install (FILES ${GENERATED_PROTOBUF_HEADERS}
  DESTINATION include/perls-protobuf
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
  )

# CLEAN UP
#file (GLOB CLEAN_CC "*.cc")
#file (GLOB CLEAN_H "*.h")
#file (GLOB CLEAN_PY "*.py")
#set (CLEAN ${CLEAN_CC} ${CLEAN_H} ${CLEAN_PY})
#set_directory_properties (PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${CLEAN}")

