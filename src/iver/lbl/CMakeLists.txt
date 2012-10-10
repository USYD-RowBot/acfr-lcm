# EXTERNAL DEPS
perlsx_boost ()
perlsx_eigen3 ()

perls_add_executable (lbl-nav main.cpp lbl.cpp lbl.h)
perls_target_link_libraries (lbl-nav perls-lcmtypes perls-common
    ${PERLSX_EIGEN3}
    ${PERLSX_BOOST}
    )
