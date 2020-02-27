#!/bin/bash

HOSTNAME=`hostname -s`
echo Starting lcm for $HOSTNAME
PATH=$PATH:/home/johnsumskas/robotx/acfr-lcm/build/bin
#export GENICAM_GENTL32_PATH=/home/auv/git/acfr-lcm/third-party/build/Vimba_1_4/VimbaGigETL/CTI/x86_32bit
#export GENICAM_GENTL64_PATH=/home/auv/git/acfr-lcm/third-party/build/Vimba_1_4/VimbaGigETL/CTI/x86_64bit
bot-procman-deputy 2> /dev/null &
sleep 1
bot-procman-sheriff -n --on-script-complete exit /home/johnsumskas/robotx/acfr-lcm/config/procman/procman-wamv.cfg wamv
