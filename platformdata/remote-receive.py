######################################################################
# This file handles all the platform pose updates.
#
# There are some TODOS listed throughout the file with instructions.
# See documentation below for more info.
#
# Contact me if you have any questions.
#
# Author:
#   Ariell Friedman
#   ariell.friedman@gmail.com
#   25 AUG 2014
#
######################################################################

from random import random, randint
import time
import threading
import math
import json, requests

# This global dictionary stores all the platform information updates
platformdata = {}

######################################################################
# Start threads for platform updates
# These are currently fake threads that update the vehicle
# poses. TODO: Make them real or replace them with something similar!
######################################################################
def init_platformdata_threads():
    return


def init_push_data(configfile):
    return

######################################################################
# Get data for a specific platform
# The global variable platformdata is updated by another process/thread.
# This function simply reads the output for a specific platform
######################################################################
def get_platformdata(platform):
    global platformdata
    data = platformdata[platform]  # get data
    data['curts'] = int(time.time())    # gets curts from remote post
    if (data['curts']-data['msgts']) > 30:
        data['pose']['uncertainty'] = (data['curts']-data['msgts'])*0.5
    return data


def set_platformdata(platform=None, data={}):
    global platformdata
    if platform is None:
        platformdata = json.loads(data)
    else:
        platformdata[platform] = json.loads(data)
    return



######################################################################
# Parse mission file
# This platform
# Outputs latlngs and origin from mission file
# TODO: parse actual mission file
######################################################################
def parse_mission (filepath, origin=[0, 0]):
    # TODO: parse mission file and origin and return in LAT/LON
    # filepath will contain the filepath to the mission file
    url = "{}/get_mission?filepath={}&olat={}&olon={}".format(sourceserver[0],filepath, origin[0], origin[1])
    data = json.loads(requests.get(url=url).text)
    #print data
    latlngs = data["latlngs"]
    origin = data["origin"]

    return latlngs, origin


######################################################################
# Set waypoint
# TODO: make real
######################################################################
def send_to_platform(platform, lat, lon):

    response = "This feature has not been implemented yet!!!"
    return platform, response
