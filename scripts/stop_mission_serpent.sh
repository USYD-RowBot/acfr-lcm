#!/bin/bash
# stop triggering
/home/auv/git/acfr-lcm/src/acfr/python/camtrigger.py stop
/home/auv/git/acfr-lcm/src/acfr/python/cam_control.py stop
echo "Triggering and capture stopped"

#kill infinite watchman
#ps -e|grep infinite | grep -v grep | awk '{print $1}'|xargs kill
#echo "Killed infinite_watchman"

#kill lcm-logger
pkill lcm-logger
result=$?
if [ $result -eq 1 ]; then
    echo "lcm-logger not running."
elif [ $result -eq 0 ]; then
    echo "Killed lcm-logger."
else
    echo "Unknown output of pkill ($result)"
fi
