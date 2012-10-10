# Author Rafa GaitÃ¡n <rgaitan@mirage-tech.com>

FIND_PATH(LAS_INCLUDE_DIR liblas/liblas.hpp 
    ${LAS_DIR}/include
    $ENV{LAS_DIR}/include
    $ENV{LAS_DIR}
    $ENV{LASDIR}/include
    $ENV{LASDIR}
    $ENV{LAS_ROOT}/include
    NO_DEFAULT_PATH
)

FIND_PATH(LAS_INCLUDE_DIR liblas/liblas.hpp)

MACRO(FIND_LAS_LIBRARY MYLIBRARY MYLIBRARYNAME)

    FIND_LIBRARY("${MYLIBRARY}_DEBUG"
        NAMES "${MYLIBRARYNAME}${CMAKE_DEBUG_POSTFIX}"
        PATHS
	${LAS_DIR}/lib/Debug
        ${LAS_DIR}/lib64/Debug
        ${LAS_DIR}/lib
        ${OSG_DIR}/lib64
        $ENV{LAS_DIR}/lib/debug
        $ENV{LAS_DIR}/lib64/debug
        $ENV{LAS_DIR}/lib
        $ENV{LAS_DIR}/lib64
        $ENV{LAS_DIR}
        $ENV{LASDIR}/lib
        $ENV{LASDIR}/lib64
        $ENV{LASDIR}
        $ENV{LAS_ROOT}/lib
        $ENV{LAS_ROOT}/lib64
        NO_DEFAULT_PATH
    )

    FIND_LIBRARY("${MYLIBRARY}_DEBUG"
        NAMES "${MYLIBRARYNAME}${CMAKE_DEBUG_POSTFIX}"
        PATHS
        ~/Library/Frameworks
        /Library/Frameworks
        /usr/local/lib
        /usr/local/lib64
        /usr/lib
        /usr/lib64
        /sw/lib
        /opt/local/lib
        /opt/csw/lib
        /opt/lib
        [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;LAS_ROOT]/lib
        /usr/freeware/lib64
    )
    
    FIND_LIBRARY(${MYLIBRARY}
        NAMES ${MYLIBRARYNAME}
        PATHS
	    ${LAS_DIR}/lib/Release
        ${LAS_DIR}/lib64/Release
        ${LAS_DIR}/lib
        ${LAS_DIR}/lib64
        $ENV{LAS_DIR}/lib/Release
        $ENV{LAS_DIR}/lib64/Release
        $ENV{LAS_DIR}/lib
        $ENV{LAS_DIR}/lib64
        $ENV{LAS_DIR}
        $ENV{LASDIR}/lib
        $ENV{LASDIR}/lib64
        $ENV{LASDIR}
        $ENV{LAS_ROOT}/lib
        $ENV{LAS_ROOT}/lib64
        NO_DEFAULT_PATH
    )

    FIND_LIBRARY(${MYLIBRARY}
        NAMES ${MYLIBRARYNAME}
        PATHS
        ~/Library/Frameworks
        /Library/Frameworks
        /usr/local/lib
        /usr/local/lib64
        /usr/lib
        /usr/lib64
        /sw/lib
        /opt/local/lib
        /opt/csw/lib
        /opt/lib
        [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;LAS_ROOT]/lib
        /usr/freeware/lib64
    )
    
    IF( NOT ${MYLIBRARY}_DEBUG)
        IF(MYLIBRARY)
            SET(${MYLIBRARY}_DEBUG ${MYLIBRARY})
         ENDIF(MYLIBRARY)
    ENDIF( NOT ${MYLIBRARY}_DEBUG)
           
ENDMACRO(FIND_LAS_LIBRARY LIBRARY LIBRARYNAME)

FIND_LAS_LIBRARY(LAS_LIBRARY las)
FIND_LAS_LIBRARY(LAS_C_LIBRARY las_c)

SET(LAS_LIBRARIES
	${LAS_LIBRARY}
	${LAS_C_LIBRARY}
)

SET(LAS_LIBRARIES_DEBUG
	${LAS_LIBRARY_DEBUG}
	${LAS_C_LIBRARY_DEBUG}
)

# handle the QUIETLY and REQUIRED arguments and set
# LAS_FOUND to TRUE as appropriate
INCLUDE( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( LAS DEFAULT_MSG LAS_INCLUDE_DIR LAS_LIBRARIES)

mark_as_advanced (
  LAS_C_LIBRARY
  LAS_C_LIBRARY_DEBUG
  LAS_INCLUDE_DIR
  LAS_LIBRARY
  LAS_LIBRARY_DEBUG
  )
