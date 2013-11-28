# Install script for directory: /home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python

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
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/build_prefix.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/build_prefix.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/printf_t.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/printf_t.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_cmd_t.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/sheriff_cmd_t.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/sheriff.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/orders_t.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/orders_t.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/info_t.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/info_t.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_gtk/sheriff_gtk.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_gtk" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/sheriff_gtk/sheriff_gtk.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_gtk/sheriff_dialogs.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_gtk" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/sheriff_gtk/sheriff_dialogs.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_gtk/command_treeview.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_gtk" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/sheriff_gtk/command_treeview.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_gtk/__init__.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_gtk" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/sheriff_gtk/__init__.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_gtk/hosts_treeview.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_gtk" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/sheriff_gtk/hosts_treeview.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_gtk/command_model.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_gtk" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/sheriff_gtk/command_model.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_gtk/command_console.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_gtk" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/sheriff_gtk/command_console.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_config.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/sheriff_config.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/__init__.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/__init__.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/discovery_t.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/discovery_t.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/deputy_cmd_t.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/deputy_cmd_t.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/python2.7/dist-packages/bot_procman/sheriff_script.py")
FILE(INSTALL DESTINATION "/usr/local/lib/python2.7/dist-packages/bot_procman" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/src/bot_procman/sheriff_script.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE PROGRAM FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/pod-build/python/bot-procman-sheriff")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bot_procman" TYPE FILE FILES "/home/lash/git/acfr_lcm/third-party/libbot2/bot2-procman/python/procman-sheriff.glade")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

