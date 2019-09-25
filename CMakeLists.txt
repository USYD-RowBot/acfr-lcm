project (perls)

# CMAKE TWEAKS
#========================================================================
cmake_minimum_required (VERSION 2.8)
set (CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE)
set (FIND_LIBRARY_USE_LIB64_PATHS 1)

# INCLUDE LOCAL MODULES
#========================================================================
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules")
file (GLOB MODULES "${CMAKE_MODULE_PATH}/*.cmake")
foreach (MODULE ${MODULES})
  if (NOT ${MODULE} MATCHES "perlsx_.*cmake$") # print all modules *other than* perlsx_XXX.cmake
    message (STATUS "Using local module ${MODULE}")
  endif ()
  include (${MODULE})
endforeach ()


# OUTPUT DIRECTORIES
#========================================================================
# set where files should be output locally
set (LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
if (NOT IS_DIRECTORY ${LIBRARY_OUTPUT_PATH})
  file (MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})
endif ()

set (EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
if (NOT IS_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
  file (MAKE_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
endif ()

set (INCLUDE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/include)
if (NOT IS_DIRECTORY ${INCLUDE_OUTPUT_PATH})
  file (MAKE_DIRECTORY ${INCLUDE_OUTPUT_PATH})
endif ()

set (PKG_CONFIG_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/pkgconfig)
if (NOT IS_DIRECTORY ${PKG_CONFIG_OUTPUT_PATH})
  file (MAKE_DIRECTORY ${PKG_CONFIG_OUTPUT_PATH})
endif ()

set (JAVA_OUTPUT_PATH ${CMAKE_BINARY_DIR}/share/java)
if (NOT IS_DIRECTORY ${JAVA_OUTPUT_PATH})
  file (MAKE_DIRECTORY ${JAVA_OUTPUT_PATH})
endif ()

perls_python_get_version (PYTHON_VERSION)
set (PYTHON_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/python${PYTHON_VERSION}/dist-packages)
if (NOT IS_DIRECTORY ${PYTHON_OUTPUT_PATH})
  file (MAKE_DIRECTORY ${PYTHON_OUTPUT_PATH})
endif ()

set (CONFIG_SOURCE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config)
set (CONFIG_RELATIVE_PATH ../share/perls)

# set where files should be installed to
set (LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
set (EXECUTABLE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin)
set (INCLUDE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/include)
set (PKG_CONFIG_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)
set (JAVA_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/share/java)
set (PYTHON_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_VERSION}/dist-packages)
set (CONFIG_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/share/perls)

# be nice and add our own include files to the search path
include_directories (${INCLUDE_OUTPUT_PATH})

include_directories ( 
  /usr/include/uf-3.2
  /usr/include/glib-2.0
  ) 


# CUSTOMIZE HOW LIBRARIES ARE BUILT
#=======================================================================
option (BUILD_SHARED_LIBS "Build shared libraries?" ON)
mark_as_advanced (FORCE BUILD_SHARED_LIBS)

# use, i.e. don't skip the full RPATH for the build tree
set (CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set (CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
set (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list (FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if ("${isSystemDir}" STREQUAL "-1")
  set (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif ()

# LCM RELATED
#========================================================================
find_program (LCM_GEN_EXE lcm-gen
  DOC "Fullpath to lcm-gen executable."
  )
mark_as_advanced (FORCE LCM_GEN_EXE)
if (NOT LCM_GEN_EXE)
  message (SEND_ERROR "lcm-gen not found; did you build lcm in third-party?")
endif ()

set (PERLS_LCMDEFS_DIR "${PROJECT_SOURCE_DIR}/lcmdefs")
include (${PERLS_LCMDEFS_DIR}/PACKAGES.cmake)

# THIRD-PARTY
#========================================================================
set (THIRD_PARTY_DIR "${PROJECT_SOURCE_DIR}/third-party")

# GO FORTH!
#========================================================================
set (FIRST_PASS_COMPLETE FALSE CACHE INTERNAL "First time?")
add_subdirectory (src)
#add_subdirectory (doxygen)
set (FIRST_PASS_COMPLETE TRUE CACHE INTERNAL "First time?")

# CONFIG STUFF
#========================================================================
add_custom_target (config ALL
  COMMAND mkdir -p ${EXECUTABLE_OUTPUT_PATH}/${CONFIG_RELATIVE_PATH}
  COMMAND ln -sf ${CONFIG_SOURCE_PATH} ${EXECUTABLE_OUTPUT_PATH}/${CONFIG_RELATIVE_PATH}/.
)

install (DIRECTORY ${CONFIG_SOURCE_PATH}
  DESTINATION ${EXECUTABLE_INSTALL_PATH}/${CONFIG_RELATIVE_PATH}/.
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ WORLD_WRITE
  DIRECTORY_PERMISSIONS  OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE  GROUP_EXECUTE WORLD_READ WORLD_WRITE WORLD_EXECUTE
  PATTERN ".svn" EXCLUDE
  )



# UNINSTALL TARGET
#========================================================================
###create the uninstall script
set (unfile ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
file (WRITE ${unfile} "IF(NOT EXISTS \"install_manifest.txt\")\n")
file (APPEND ${unfile} "MESSAGE(\"FATAL_ERROR Cannot find \\\"install manifest\\\": install_manifest.txt\")\n")
file (APPEND ${unfile} "ENDIF(NOT EXISTS \"install_manifest.txt\")\n")
file (APPEND ${unfile} "FILE(READ \"install_manifest.txt\" files)\n")
file (APPEND ${unfile} "STRING(REGEX REPLACE \"\\n\" \";\" files \"\${files}\")\n")
file (APPEND ${unfile} "FOREACH(file \${files})\n")
file (APPEND ${unfile} "  MESSAGE(STATUS \"Uninstalling \\\"\${file}\\\"\")\n")
file (APPEND ${unfile} "  IF(EXISTS \"\${file}\")\n")
file (APPEND ${unfile} "    EXEC_PROGRAM(\n")
file (APPEND ${unfile} "      \"\${CMAKE_COMMAND}\" ARGS \"-E remove \\\"\${file}\\\"\"\n")
file (APPEND ${unfile} "    OUTPUT_VARIABLE rm_out\n")
file (APPEND ${unfile} "      RETURN_VALUE rm_retval\n")
file (APPEND ${unfile} "      )\n")
file (APPEND ${unfile} "    IF(\"\${rm_retval}\" STREQUAL 0\)\n")
file (APPEND ${unfile} "    ELSE(\"\${rm_retval}\" STREQUAL 0\)\n")
file (APPEND ${unfile} "      MESSAGE(FATAL_ERROR \"Problem when removing \\\"\${file}\\\"\")\n")
file (APPEND ${unfile} "    ENDIF(\"\${rm_retval}\" STREQUAL 0)\n")
file (APPEND ${unfile} "  ELSE(EXISTS \"\${file}\")\n")
file (APPEND ${unfile} "    MESSAGE(STATUS \"File \\\"\${file}\\\" does not exist.\")\n")
file (APPEND ${unfile} "  ENDIF(EXISTS \"\${file}\")\n")
file (APPEND ${unfile} "ENDFOREACH(file)\n")
########################
add_custom_target (uninstall
 "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
###########################
