perlsx_glib ()

find_package(LibXML++ REQUIRED)

set(LIBXMLPP_LIBS  ${LIBXMLPP_LIBRARIES})
include_directories(${LibXML++_INCLUDE_DIRS})

perls_add_executable (global-planner global_planner.cpp mission.cpp  ZamboniePath.cpp SpiralPath.cpp SpiralInwardPath.cpp GotoAndCirclePath.cpp GotoPath.cpp LegPath.cpp)
perls_target_link_libraries (global-planner perls-lcmtypes++ perls-common ${PERLSX_GLIB} ${LibXML++_LIBRARIES})


