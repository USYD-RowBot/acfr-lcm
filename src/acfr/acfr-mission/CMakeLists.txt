find_package(LibXML++ REQUIRED)

set(LIBXMLPP_LIBS  ${LIBXMLPP_LIBRARIES})
include_directories(${LibXML++_INCLUDE_DIRS})

set (SRC 
    mission.cpp             mission.hpp   
                            MissionPrimitive.hpp  
                            mission_command.hpp 
                            waypoint.hpp    
                            Command.h
    ZamboniePath.cpp        ZamboniePath.h  
    SpiralPath.cpp          SpiralPath.h 
    SpiralInwardPath.cpp    SpiralInwardPath.h  
    GotoAndCirclePath.cpp   GotoAndCirclePath.h  
    GotoPath.cpp            GotoPath.h 
    LegPath.cpp             LegPath.h  
    CoveragePath.cpp        CoveragePath.h
)

       

perls_add_library(acfr-mission
    SHARED ${SRC}
    )
 
perls_target_link_libraries(acfr-mission
	perls-lcmtypes++ perls-common ${PERLSX_GLIB} ${LibXML++_LIBRARIES}
)

include_directories(/usr/include/python2.7 ${PROJECT_SOURCE_DIR}/acfr/acfr-mission)
SET(PYTHON_SITE_PACKAGES_DIR /usr/lib/python2.7/dist-packages)
ADD_SIP_PYTHON_MODULE(missionXML ./missionXML.sip ${LibXML++_LIBRARIES} acfr-mission)
