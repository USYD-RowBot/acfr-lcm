#!/bin/bash

HOSTNAME=`hostname -s`
echo Starting lcm for $HOSTNAME
PATH=$PATH:/home/drop/workspace/acfr-lcm/src/acfr/python;/home/drop/workspace/acfr-lcm/build/bin
export GENICAM_GENTL32_PATH=/home/drop/workspace/acfr-lcm/third-party/build/Vimba_1_4/VimbaGigETL/CTI/x86_32bit
export GENICAM_GENTL64_PATH=/home/drop/third_party/Vimba_2_1/VimbaGigETL/CTI/x86_64bit/
bot-procman-deputy 2> /dev/null &
bot-procman-sheriff -n --on-script-complete exit /home/drop/workspace/acfr-lcm/config/procman/procman-$HOSTNAME.cfg $HOSTNAME

