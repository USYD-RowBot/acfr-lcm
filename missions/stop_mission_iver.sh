#!/bin/bash
pkill lcm-logger
pkill acfr-cam-logger
/home/auv/git/acfr_lcm/src/acfr/python/cam_control.py stop
