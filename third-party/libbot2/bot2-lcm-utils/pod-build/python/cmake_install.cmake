# Install script for directory: /home/navid/proj/acfr/acfr_lcm/third-party/libbot2New/bot2-lcm-utils/python

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/navid/proj/acfr/acfr_lcm/third-party/build")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/navid/proj/acfr/acfr_lcm/third-party/build/lib/python2.7/dist-packages/bot_log2mat/log_to_mat.py")
FILE(INSTALL DESTINATION "/home/navid/proj/acfr/acfr_lcm/third-party/build/lib/python2.7/dist-packages/bot_log2mat" TYPE FILE FILES "/home/navid/proj/acfr/acfr_lcm/third-party/libbot2New/bot2-lcm-utils/python/src/bot_log2mat/log_to_mat.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/navid/proj/acfr/acfr_lcm/third-party/build/lib/python2.7/dist-packages/bot_log2mat/scan_for_lcmtypes.py")
FILE(INSTALL DESTINATION "/home/navid/proj/acfr/acfr_lcm/third-party/build/lib/python2.7/dist-packages/bot_log2mat" TYPE FILE FILES "/home/navid/proj/acfr/acfr_lcm/third-party/libbot2New/bot2-lcm-utils/python/src/bot_log2mat/scan_for_lcmtypes.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/navid/proj/acfr/acfr_lcm/third-party/build/lib/python2.7/dist-packages/bot_log2mat/__init__.py")
FILE(INSTALL DESTINATION "/home/navid/proj/acfr/acfr_lcm/third-party/build/lib/python2.7/dist-packages/bot_log2mat" TYPE FILE FILES "/home/navid/proj/acfr/acfr_lcm/third-party/libbot2New/bot2-lcm-utils/python/src/bot_log2mat/__init__.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE PROGRAM FILES "/home/navid/proj/acfr/acfr_lcm/third-party/libbot2New/bot2-lcm-utils/pod-build/python/bot-log2mat")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

