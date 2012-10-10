# SOURCE FILES
set (SRC 
                           ascii.h
  bot_util.c               bot_util.h
  cache.c                  cache.h
                           cinttypes.h
  daemon.c                 daemon.h
  dfs.c                    dfs.h
                           error.h
  generic_sensor_driver.c  generic_sensor_driver.h
  getopt.c                 getopt.h
  glib_util.c              glib_util.h
  lcm_util.c               lcm_util.h
  logbook.c                logbook.h
  magic.c                  magic.h
  nmea.c                   nmea.h
  serial.c                 serial.h
  stdio_util.c             stdio_util.h
  textread.c               textread.h
  timestamp.c              timestamp.h
  timeutil.c               timeutil.h
  udp.c                    udp.h		  
                           units.h
  unix.c                   unix.h
  )

# EXTERNAL DEPS
perlsx_bot2 ()
perlsx_glib ()
perlsx_gsl ()
perlsx_lcm ()
perlsx_m ()

# BUILD LIBRARY
perls_add_library (perls-common ${SRC})
perls_target_link_libraries (perls-common perls-lcmtypes
  ${PERLSX_BOT2}
  ${PERLSX_GLIB}
  ${PERLSX_GSL}
  ${PERLSX_LCM}
  ${PERLSX_M}
  ${PERLSX_PTHREAD}
  )

