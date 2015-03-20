#!/bin/bash

HOSTNAME=`hostname -s`
echo 'Starting lcm for $HOSTNAME'
PATH=$PATH:/home/auv/git/acfr_lcm/build/bin
export GENICAM_GENTL32_PATH=/home/auv/git/acfr_lcm/third-party/build/Vimba_1_3/AVTGigETL/CTI/x86_32bit
bot-procman-deputy 2> /dev/null &
bot-procman-sheriff -n --on-script-complete exit /home/auv/git/acfr_lcm/config/procman/procman-$HOSTNAME.cfg $HOSTNAME

