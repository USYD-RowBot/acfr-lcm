#!/bin/bash

# start_mission.sh [mission_name]
# starts the image logger

LCMBINDIR="/home/auv/git/acfr-lcm/build/bin"
LOGBASEDIR="/media/data"
TIMESTAMP=`date --utc +%Y%m%d_%H%M%S`

# Get mission name.
MISSION_NAME=r$TIMESTAMP\_${1}
LOGDIR=${LOGBASEDIR}/${MISSION_NAME}
IMAGE_LOGDIR=$LOGDIR/i$TIMESTAMP/

# create the directory for the images
mkdir -p $LOGDIR
mkdir -p $IMAGE_LOGDIR
export LCM_MISSION_DIR=$LOGDIR

# kill any loggers that may be running
killall -q lcm-logger

# copy the curent config file into the data directory
cp /home/auv/git/acfr-lcm/config/nga.cfg $LOGDIR
cp /home/auv/git/acfr-lcm/config/slam_nga.cfg $LOGDIR
cp /tmp/magnetic_variation.cfg $LOGDIR

/home/auv/git/acfr-lcm/src/acfr/python/cam_control.py path $IMAGE_LOGDIR
/home/auv/git/acfr-lcm/src/acfr/python/cam_control.py start
lcm-logger -v -c "PROSILICA_..16" $LOGDIR/$MISSION_NAME.lcm > /dev/null 2> /dev/null &
lcm-logger -v -c ".*RAW|PMD.*|.*STATS|PARAM.*|PROSILICA_..16|LCM.*|HEARTBEAT.*" $LOGDIR/${MISSION_NAME}_short.lcm > /dev/null 2> /dev/null &
