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
sourceserver = ["http://10.23.9.211:8080"]
#destserver = "http://tracker.marine.acfr.usyd.edu.au/"
destserver = "http://10.23.9.164:8083/setall_platformdata"

######################################################################
# Start threads for platform updates
# These are currently fake threads that update the vehicle
# poses. TODO: Make them real or replace them with something similar!
######################################################################
def init_platformdata_threads():
    sendRemoteDataThread(5).start()


######################################################################
# Get data for a specific platform
# The global variable platformdata is updated by another process/thread.
# This function simply reads the output for a specific platform
######################################################################
def get_platformdata(platform):
    data = platformdata[platform]  # get data
    data['curts'] = time.time()    # add curr ts
    return data


def setall_platformdata(data):
    platformdata = data

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
    print data
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


######################################################################
# The thread classes below spoof fake random nav data for fake platforms
# It provides an example of the data structures that are required to feed real updates to the map
# TODO: create a similar class that fills out the data structure with real data
# It may be cleaner to keep the class another file that is imported into this one
class sendRemoteDataThread (threading.Thread):
    def __init__(self, delay, targets=['AUVSTAT.SIRIUS', 'FALKOR', 'USBL_FIX.SIRIUS']):
        threading.Thread.__init__(self)
        self.delay = delay
        self.daemon = True  # run in daemon mode to allow for ctrl+C exit

    def run (self):
        platforms = {
            "AUVSTAT.SIRIUS": "{}/get_platformdata?platform=AUVSTAT.SIRIUS".format(sourceserver[0]),
            "FALKOR": "{}/get_platformdata?platform=FALKOR".format(sourceserver[0]),
            "USBL_FIX.SIRIUS": "{}/get_platformdata?platform=USBL_FIX.SIRIUS".format(sourceserver[0])
        }

        while(1) :
            for key in platforms:
                try:
                    platformdata[key] = json.loads(requests.get(url=platforms[key]).text)
                    print "Received data for: {}".format(platforms[key])
                except:
                    print "ERROR!!! getting data for: {}".format(platforms[key])

            print "Sending data to {}".format(destserver)
            payload = {'platformdata': json.dumps(platformdata)}
            #payload = {'platformdata': platformdata}
            r = requests.post(destserver, data=payload)

            time.sleep(self.delay)
