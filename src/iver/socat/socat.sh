#!/usr/bin/env bash

# os-compass
#options="raw,echo=0,b19200"
#socat /dev/ttyS4,$options /dev/ttyS0,$options

# microstrain
#options="raw,echo=0,b115200"
#socat /dev/ttyS10,$options /dev/ttyS0,$options

EXPECTED_ARGS=2

if [ $# -ne $EXPECTED_ARGS ]
then
  echo "Usage: `basename $0` device baud"
  echo "forwards a serial port on the linux stack to COM1 on the wafer"
  echo ""
  echo "examples:"
  echo "`basename $0` ttyS4  b115200  #os-compass"
  echo "`basename $0` ttyS6  b19200   #modem"
  echo "`basename $0` ttyS10 b115200  #microstrain"
  exit -1
fi

options="raw,echo=0"
socat /dev/$1,$options,$2 /dev/ttyS0,$options,$2
