#!/bin/bash

# Script to execute on login
# Add the following to the .bashrc
#
# if [ "/dev/tty1" == "$(tty)" ]; then
#     bash /home/auv/diver_onlogin.sh
# fi

#setfont /usr/share/consolefonts/Lat2-TerminusBold32x16.psf.gz
setfont /usr/share/consolefonts/Lat15-TerminusBold32x16.psf.gz
bash /home/auv/start_lcm.sh
python /home/auv/experimental-scripts/diver-screen.py
