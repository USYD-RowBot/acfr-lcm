perlsx_glib ()

find_package(LibXML++ REQUIRED)

set(LIBXMLPP_LIBS  ${LIBXMLPP_LIBRARIES})
include_directories(${LibXML++_INCLUDE_DIRS})

perls_add_executable (mission-control main.cpp main_control.cpp mission_control.cpp mission.cpp mission_primitives.cpp)
perls_target_link_libraries (mission-control perls-lcmtypes++ perls-common ${PERLSX_GLIB} ${LibXML++_LIBRARIES})


