add_executable (waveglider-payload main.c checksum.c messages.c)
target_link_libraries (waveglider-payload perls-lcmtypes perls-common acfr-common)
