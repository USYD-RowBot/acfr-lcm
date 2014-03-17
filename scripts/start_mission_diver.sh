#!/bin/bash
#
# USAGE: start_mission_diver.sh
#
# Sets up and runs the missions for the diver rig.
# Sets up archival logging directories and starts
# LCM logger.
#

LCMROOT="/home/auv/git/acfr_lcm"
ORIGINDIR="$LCMROOT/config/origins/d2"

# Get parameters from the generated config file that is output from 
# the "new_origin_diver.sh" script.
# Sets the variables: 
#       - ORIGIN_NAME
#       - ORIGIN_LAT
#       - ORIGIN_LNG
eval "`cat $ORIGINDIR/current_origin.conf`"

# Get mission name and increment counter
COUNTFILE="$ORIGINDIR/$ORIGIN_NAME.count"
if [ -f $COUNTFILE ]; then
    COUNT=$((`cat $COUNTFILE` + 1))
else
    COUNT=1
fi
echo $COUNT > $COUNTFILE

# Set current timestamp
TIMESTAMP=`date --utc +%Y%m%d_%H%M%S`

# Get mission name.
MISSION_NAME=r$TIMESTAMP\_${ORIGIN_NAME}_d2_`printf "%03d" $COUNT`

# TODO: this needs to be automated...
DEPTH_TARE="0"

# Check that heartbeat is up, using this as indicative of sensors. 
if [[ ! `pgrep cam-trigger` || ! `pgrep heartbeat` ]]; then
    echo "Error: heartbeam or cam-trigger LCM process not running, are sensors and cameras active?"
    exit 1
fi

# Create a timestamped logging (raw) directory.  
LOGDIR=/media/data/$MISSION_NAME
mkdir $LOGDIR


# Create a timestamped image logging (idir) directory
ILOGDIR=$LOGDIR/i$TIMESTAMP/
mkdir $ILOGDIR

# Create mission config file and put it into the mission directory
$LCMROOT/scripts/generate_mission_cfg.sh $ORIGIN_LAT $ORIGIN_LNG $DEPTH_TARE > $LOGDIR/mission.cfg

# Archive vehicle configuration and lcm definitions.
rsync -L $LCMROOT/config/diver.cfg $LOGDIR/
rsync -ak --exclude=".svn" $LCMROOT/lcmdefs $LOGDIR/

# Restart the LCM-logger, directing it to log data into the timestamped
# directory.  
killall -q lcm-logger
lcm-logger -v -c PROSILICA_..16 $LOGDIR/$MISSION_NAME.lcm > /dev/null &

# Restart the cam-logger, directing it to log data into the idir
#killall -q acfr-cam-logger 
#$LCMROOT/build/bin/acfr-cam-logger -c "PROSILICA_..16" -o $ILOGDIR > /dev/null 2> /dev/null &
python $LCMROOT/src/acfr/python/cam_control.py path $ILOGDIR &
python $LCMROOT/src/acfr/python/cam_control.py start &

# Send camera trigger START message
python $LCMROOT/src/acfr/python/camtrigger.py start &

# Mission is now running!
