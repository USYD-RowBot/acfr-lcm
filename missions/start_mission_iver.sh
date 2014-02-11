#!/bin/bash

# start_mission.sh [mission_name]
# starts the image logger

LCMBINDIR="/home/auv/git/acfr_lcm/build/bin"
LOGBASEDIR="/media/data/"
TIMESTAMP=`date --utc +%Y%m%d_%H%M%S`

# Get mission name.
MISSION_NAME=r$TIMESTAMP\_${1}
LOGDIR=$LOGBASEDIR/$MISSION_NAME
IMAGE_LOGDIR=$LOGDIR/i$TIMESTAMP/

# create the directory for the images
mkdir -p $LOGDIR
mkdir -p $IMAGE_LOGDIR

# kill any loggers that may be running
killall -q lcm-logger
killall -q acfr-cam-logger

# copy the curent config file into the data directory
cp /home/auv/git/acfr_lcm/config/iverACFR.cfg $LOGDIR


$LCMBINDIR/acfr-cam-logger -c "PROSILICA_..16" -o $IMAGE_LOGDIR > /dev/null 2> /dev/null &
lcm-logger -v -c "PROSILICA_..16" $LOGDIR/$MISSION_NAME.lcm > /dev/null 2> /dev/null &
