#!/bin/bash
#
# USAGE: stop_mission_diver.sh
#
# LCM logger and cameras.
# Restarts LCM logger pointed to idle_logs directory.

LCMROOT="/home/auv/git/acfr_lcm"


# Send camera trigger STOP message
python $LCMROOT/src/acfr/python/camtrigger.py stop &

# Restart LCM logger directing output to idle_logs directory.
#killall -q acfr-cam-logger  # kill camera log
python $LCMROOT/src/acfr/python/cam_control.py stop &

killall -q lcm-logger       # kill lcm logger
TIMESTAMP=`date --utc +%Y%m%d_%H%M%S`
lcm-logger -v -c PROSILICA_..16 -i -s /media/data/idle_logs/$TIMESTAMP.lcm > /dev/null &

