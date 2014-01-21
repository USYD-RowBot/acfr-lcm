add_executable (sirius-control main.c pid.c trajectory.c controller.c control_utils.c timer.c)
target_link_libraries (sirius-control perls-lcmtypes perls-common)
