# SOURCE FILES
set (SRC main.cpp 
    jausocu.cpp                         jausocu.h
    experimentalMessageHeaders.h
    )
file (GLOB EXP_MSG_SRC "experimental/*.c")
file (GLOB EXP_MSG_INC "experimental/*.h")
    #queryCurvatureMessage.c             queryCurvatureMessage.h
    #queryErrorCountMessage.c            queryErrorCountMessage.h
    #queryErrorMessage.c                 queryErrorMessage.h
    #querySignalsMessage.c               querySignalsMessage.h
    #queryWheelSpeedsMessage.c           queryWheelSpeedsMessage.h
    #reportCurvatureMessage.c            reportCurvatureMessage.h
    #reportErrorCountMessage.c           reportErrorCountMessage.h
    #reportErrorMessage.c                reportErrorMessage.h
    #reportSignalsMessage.c              reportSignalsMessage.h
    #reportWheelSpeedsMessage.c          reportWheelSpeedsMessage.h
    #setMotionProfileMessage.c           setMotionProfileMessage.h
    #setSignalsMessage.c                 setSignalsMessage.h

    
# EXTERNAL DEPS
perlsx_openjaus ()

# BUILD EXE
perls_add_executable (lcm-jaus-ocu ${SRC} ${EXP_MSG_SRC} ${EXP_MSG_INC})
perls_target_link_libraries (lcm-jaus-ocu perls-lcmtypes perls-common pthread
  ${PERLSX_JAUS}
  ${PERLSX_OPENJAUS}
)
