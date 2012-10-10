# LIBRARIES
perls_add_folder (lcmtypes "Build lcmtypes libraries? (c,c++,java,python)" ON )
perls_add_folder (perls-common "Build common utils library?" ON)
perls_add_folder (perls-est "Build estimation utils library?" ON)
perls_add_folder (perls-math "Build math utils library?" ON)
perls_add_folder (BotParam "Build python bot param package?" ON)
perls_add_folder (BotCore "Build python bot core package?" ON)


# BINARIES
perls_add_folder (heartbeat "Build heartbeat timer process?" ON)
perls_add_folder (logbook "Build logbook tool?" ON)
perls_add_folder (lcmlog-export "Build lcmlog-export tool?" ON)
perls_add_folder (lcm-sink "Build LCM sink tool?" ON)
perls_add_folder (spy "Build LCM spy tool?" ON)
