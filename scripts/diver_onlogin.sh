#!/bin/bash

# Script to execute on login
# Add the following to the .bashrc
#
# if [ "/dev/tty1" == "$(tty)" ]; then
#     bash /home/auv/diver_onlogin.sh
# fi

LCMROOT="/home/auv/git/acfr_lcm"

#setfont /usr/share/consolefonts/Lat2-TerminusBold32x16.psf.gz
setfont /usr/share/consolefonts/Lat15-TerminusBold32x16.psf.gz
bash $LCMROOT/scripts/start_lcm_diver.sh
python $LCMROOT/src/acfr/python/diver-screen.py
