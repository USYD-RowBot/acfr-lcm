# perls_spy.jar
#==========================================================================
set (SRC
  perls_spy/DegViewerPlugin.java
  perls_spy/RawPlugin.java
  perls_spy/ImagePlugin.java 
)

perls_java_add_library (perls_spy.jar ${SRC})
perls_java_target_link_libraries (perls_spy.jar perls_lcmtypes.jar)
add_dependencies (perls_spy.jar perls_lcmtypes.jar)



# spy exe
#==========================================================================
perls_bash_install_script (spy spy.in)
