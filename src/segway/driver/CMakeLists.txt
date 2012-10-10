# find iri segway_rmp200
find_package (segway_rmp200)
if (segway_RMP200_FOUND)
  include_directories (${segway_RMP200_INCLUDE_DIR})
  set (LIB_IRI_SEGWAY ${segway_RMP200_LIBRARY})
  mark_as_advanced (segway_RMP200_INCLUDE_DIR segway_RMP200_LIBRARY)
else ()
  message (SEND_ERROR "did you install iri-segway in third-party")
endif ()

# find iri comm
find_package (comm)
if (comm_FOUND)
  include_directories (${comm_INCLUDE_DIR})
  set (LIB_IRI_COMM ${comm_LIBRARY})
  mark_as_advanced (comm_INCLUDE_DIR comm_LIBRARY)
else ()
  message (SEND_ERROR "did you install iri-segway in third-party?")
endif ()

# find iri utils
find_package (iriutils)
if (iriutils_FOUND)
  include_directories (${iriutils_INCLUDE_DIR})
  set (LIB_IRI_IRIUTILS ${iriutils_LIBRARY})
  mark_as_advanced (iriutils_INCLUDE_DIR iriutils_LIBRARY)
else ()
  message (SEND_ERROR "did you install iri-segway in third-party?")
endif ()

perls_add_executable (driver main.cpp)
perls_target_link_libraries (driver perls-lcmtypes perls-common 
  ${LIB_IRI_SEGWAY}
  ${LIB_IRI_COMM}
  ${LIB_IRI_IRIUTILS}
)
