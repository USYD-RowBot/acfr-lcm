set (SRC 
    vehicleSimBase.cpp    vehicleSimBase.hpp
)

perls_add_library (simulator-common ${SRC})
perls_target_link_libraries ( simulator-common perls-lcmtypes++ perls-common bot2-param-client acfr-common )

include_directories (                                                           
       /usr/local/include/uf-3.2                                                         
)                                                                           

link_directories (/usr/local/lib)

