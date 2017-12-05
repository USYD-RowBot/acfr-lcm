#!/bin/bash
#####################################################################
# Script generates mission.cfg file and prints to stdout.
# Pipe the output to the desired file.
# 
# Usage:
#   generate_mission_cfg.sh <LAT> <LON> <DEPTH_TARE> > mission.cfg
#
# History:
#   2011-06-14      ALF         create
# 
#####################################################################

# Magnetic variation test_script
MAGVARWMM="/home/auv/git/acfr-lcm/src/acfr/python/WMM.COF"
MAGVARPROG="/home/auv/git/acfr-lcm/src/acfr/python/geomag.py"


# Check for right number of arguments, otherwise print usage.
if [ $# -ne 3 ]; then
    echo -e "
`basename $0`
Script generates mission.cfg file and prints to stdout.
Pipe the output to the desired file.

Usage: 
    `basename $0` <LAT> <LON> <DEPTH_TARE> > mission.cfg

"
    exit
fi

# Campaign Origin
ORIGIN_LAT="$1"
ORIGIN_LNG="$2"

# Depth sensor tare
DEPTH_TARE="$3"


# Calculate magnetic variation
#MAGVARDATEARG=`date --utc +%m\ %d\ %y`
MAG_VAR=`$MAGVARPROG $MAGVARWMM $ORIGIN_LAT $ORIGIN_LNG`
#MAGVAROUT=`$MAGVARTESTPROG $ORIGIN_LAT $ORIGIN_LNG 0 $MAGVARDATEARG`
#MAG_VAR=`echo $MAGVAROUT | cut -d " " -f 17`

# echo mission config
echo "# Generated automatically by 
# `basename $0` on `date --utc +%Y/%m/%d\ %H\:%M\:%S`

MAG_VAR_DATE \"`date --utc +%Y/%m/%d`\"
MAG_VAR_LAT $ORIGIN_LAT
MAG_VAR_LNG $ORIGIN_LNG
MAGNETIC_VAR_DEG $MAG_VAR
DEPTH_TARE_YSI $DEPTH_TARE
#LATITUDE $ORIGIN_LAT
#LONGITUDE $ORIGIN_LNG"


