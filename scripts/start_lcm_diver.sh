#!/bin/bash

PATH=$PATH:/home/auv/git/acfr_lcm/build/bin
bot-procman-deputy 2> /dev/null &
bot-procman-sheriff -n --on-script-complete exit /home/auv/git/acfr_lcm/config/procman/procman-diver.cfg diver

