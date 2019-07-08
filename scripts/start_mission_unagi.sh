#!/bin/bash
# Script to start image acquisition.
# Every time a new folder YYYYMMDD_hhmmss_MISSION is created

## Parameters ##
REGION_NAME=sesoko
BASE_PATH=/media/data
PLATFORM_NAME="UG"
CURRENT_DATE=$(date '+%Y%m%d_%H%M%S')

## incase mission already running
~/stop_mission.sh

LCMROOT="/home/auv/git/acfr-lcm"
ORIGIN_DIR="$LCMROOT/config/origins/$PLATFORM_NAME"
COUNTFILE="$ORIGIN_DIR/$REGION_NAME.count"
if [ -f $COUNTFILE ]; then
    COUNT=$((`cat $COUNTFILE` + 1))
else
    mkdir -p $ORIGIN_DIR
    COUNT=1
fi
echo $COUNT > $COUNTFILE
MISSION_NUMBER=`printf "%03d" $COUNT`

# Check that DIR is empty
MISSION_DIR="$BASE_PATH/r${CURRENT_DATE}_${PLATFORM_NAME}${MISSION_NUMBER}_${REGION_NAME}"
IMAGE_DIR="${MISSION_DIR}/i${CURRENT_DATE}"
# Create the DIR and all subdirectories if necessary without prompting the user
mkdir -p $MISSION_DIR
FN=$(ls -l $MISSION_DIR | wc -l)
if [ $FN -gt 1 ]; then
    echo "WARNING: Directory $MISSION_DIR is not empty, aborting mission start";
    exit
fi;
mkdir -p $IMAGE_DIR

#log lcm data
pkill lcm-logger
lcm-logger -v -c "PROSILICA_..16" $MISSION_DIR/lcmlog_$CURRENT_DATE.lcm > /dev/null 2> /dev/null &
echo "lcm-logger started in background"

#initialise camera counts
#echo 0 > $DIR/fcount.txt
#echo 0 > $DIR/acount.txt
#echo "camera counters initialised"

/home/auv/git/acfr-lcm/src/acfr/python/cam_control.py path $IMAGE_DIR

# Don't forget to start the image logging
/home/auv/git/acfr-lcm/src/acfr/python/cam_control.py start

# Uncomment to set frequency. Default 2Hz
/home/auv/git/acfr-lcm/src/acfr/python/camtrigger.py freq 1

# Uncoment to set pulse width. Default 7ms
#/home/auv/git/acfr-lcm/src/acfr/python/camtrigger.py width 7

# start triggering
/home/auv/git/acfr-lcm/src/acfr/python/camtrigger.py start
echo "Triggering started. Saving files at: $IMAGE_DIR"

# start infinite watchman in background
#echo "Starting infinite watchman... wait 30 seconds"
#sleep 30; /home/auv/infinite_watchman.sh > /dev/null &
#echo "Automatically restart wfov if cameras stop saving"
