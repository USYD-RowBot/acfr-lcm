# Install script for directory: /home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
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
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbot2-core.so.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbot2-core.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib")
    ENDIF()
  ENDFOREACH()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/pod-build/lib/libbot2-core.so.1"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/pod-build/lib/libbot2-core.so"
    )
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbot2-core.so.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbot2-core.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/pod-build/lib:/usr/local/lib:"
           NEW_RPATH "/usr/local/lib")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "${file}")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDFOREACH()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/bot_core" TYPE FILE FILES
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/ssocket.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/signal_pipe.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/ctrans.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/circular.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/set.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/small_linalg.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/camtrans.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/serial.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/lcm_util.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/bot_core.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/tictoc.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/rand_util.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/ringbuf.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/fasttrig.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/fileutils.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/rotations.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/math_util.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/ptr_circular.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/glib_util.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/gps_linearize.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/timespec.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/minheap.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/color_util.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/trans.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/timestamp.h"
    "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/src/bot_core/ppm.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-core/pod-build/lib/pkgconfig/bot2-core.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

