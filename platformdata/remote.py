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
remoteserver = ["http://144.6.227.28:8080"]

######################################################################
# Start threads for platform updates
# These are currently fake threads that update the vehicle
# poses. TODO: Make them real or replace them with something similar!
######################################################################
def init_platformdata_threads():
    getRemoteDataThread(1).start()


######################################################################
# Get data for a specific platform
# The global variable platformdata is updated by another process/thread.
# This function simply reads the output for a specific platform
######################################################################
def get_platformdata(platform):
    return platformdata[platform]

######################################################################
# Parse mission file
# This platform
# Outputs latlngs and origin from mission file
# TODO: parse actual mission file
######################################################################
def parse_mission (filepath, origin=[0, 0]):
    # TODO: parse mission file and origin and return in LAT/LON
    # filepath will contain the filepath to the mission file
    url = "{}/get_mission?filepath={}&olat={}&olon={}".format(remoteserver[0],filepath, origin[0], origin[1])
    data = json.loads(requests.get(url=url).text)
    print data
    latlngs = data["latlngs"]
    origin = data["origin"]

    return latlngs, origin


######################################################################
# Set waypoint
# TODO: make real
######################################################################
def send_waypoint(platform, lat, lon):

    response = "This feature has not been implemented yet!!!"
    return platform, response


######################################################################
# The thread classes below spoof fake random nav data for fake platforms
# It provides an example of the data structures that are required to feed real updates to the map
# TODO: create a similar class that fills out the data structure with real data
# It may be cleaner to keep the class another file that is imported into this one
class getRemoteDataThread (threading.Thread):
    def __init__(self, delay):
        threading.Thread.__init__(self)
        self.delay = delay
        self.daemon = True  # run in daemon mode to allow for ctrl+C exit

    def run (self):
        platforms = {
            "0": "{}/get_platformdata?platform=0".format(remoteserver[0]),
            "1": "{}/get_platformdata?platform=1".format(remoteserver[0]),
            "3": "{}/get_platformdata?platform=3".format(remoteserver[0])
        }

        while(1) :
            for key in platforms:
                try:
                    platformdata[key] = json.loads(requests.get(url=platforms[key]).text)
                    print "Received data for: {}".format(platforms[key])
                except:
                    print "Error getting data for: {}".format(platforms[key])
            time.sleep(self.delay)
