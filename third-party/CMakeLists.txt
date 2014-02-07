project (third-party)

cmake_minimum_required (VERSION 2.8)

set (CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE)
set (THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# include local macros
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../cmake/Modules/")
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules")
file (GLOB MODULES "${CMAKE_MODULE_PATH}/*.cmake")
foreach (MODULE ${MODULES})
  include (${MODULE})
endforeach ()

add_custom_target (third-party ALL
  COMMAND cmake ..
  )
add_custom_target (uninstall
  COMMAND cmake ..
  )
add_custom_target (clean
  COMMAND cmake ..
  )

#***********************************************************************
# add third-party library targets below in alphabetical order
#***********************************************************************

include (avt.cmake)
include (goby.cmake)
include (gpsd.cmake)
include (iri-segway.cmake)
include (isam.cmake)
include (lcm.cmake)
include (libbot2.cmake)
include (openjaus.cmake)
include (sba.cmake)
include (siftgpu.cmake)
include (small.cmake)
include (venthur-ardrone.cmake)
