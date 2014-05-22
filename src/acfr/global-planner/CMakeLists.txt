perlsx_glib ()

find_package(LibXML++ REQUIRED)

set(LIBXMLPP_LIBS  ${LIBXMLPP_LIBRARIES})
include_directories(${LibXML++_INCLUDE_DIRS})

add_library(GlobalPlanner
    SHARED
    mission.cpp ZamboniePath.cpp SpiralPath.cpp SpiralInwardPath.cpp 
    GotoAndCirclePath.cpp GotoPath.cpp LegPath.cpp CoveragePath.cpp
)
target_link_libraries(GlobalPlanner
	perls-lcmtypes++ perls-common ${PERLSX_GLIB} ${LibXML++_LIBRARIES}
)


perls_add_executable (global-planner 
	global_planner.cpp
)
perls_target_link_libraries (global-planner
	GlobalPlanner
)


perls_add_executable (GlobalPlannerTest 
	test.cpp
)
perls_target_link_libraries (GlobalPlannerTest 
	GlobalPlanner
)

install(TARGETS GlobalPlanner
     LIBRARY DESTINATION lib
)

include_directories(/usr/include/python2.7 ${PROJECT_SOURCE_DIR}/acfr/global-planner)
ADD_SIP_PYTHON_MODULE(missionXML ./missionXML.sip ${LibXML++_LIBRARIES} GlobalPlanner)
