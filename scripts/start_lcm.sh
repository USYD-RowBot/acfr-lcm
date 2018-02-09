#!/bin/bash

HOSTNAME=`hostname -s`
echo Starting lcm for $HOSTNAME
PATH=$PATH:/home/auv/git/acfr-lcm/build/bin
export GENICAM_GENTL32_PATH=/home/auv/git/acfr-lcm/third-party/build/Vimba_1_4/VimbaGigETL/CTI/x86_32bit
export GENICAM_GENTL64_PATH=/home/auv/git/acfr-lcm/third-party/build/Vimba_1_4/VimbaGigETL/CTI/x86_64bit
bot-procman-deputy 2> /dev/null &
bot-procman-sheriff -n --on-script-complete exit /home/auv/git/acfr-lcm/config/procman/procman-$HOSTNAME.cfg $HOSTNAME

