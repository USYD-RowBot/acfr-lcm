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

# This global dictionary stores all the platform information updates
platformdata = {}

######################################################################
# Start threads for platform updates
# These are currently fake threads that update the vehicle
# poses. TODO: Make them real or replace them with something similar!
######################################################################
def init_platformdata_threads():
    FakeAUVThread('0', 0.5).start()
    FakeAUVThread('3', 5).start()
    FakeShipThread('1', 1).start()
    FakeShipThread('ship2', 1).start()

# if you need to do some bookeeping on exit, flesh this out...
# it is not needed in this case, because we have deamonized threads.
def terminate_platformdata_threads():
    return 0

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

    origin = [-33.84119 , 151.25612]       # lat lon origin of mission
    latlngs = [[origin[0]+(random()-0.5)/500, origin[1]+(random()-0.5)/500],
               [origin[0]+(random()-0.5)/500, origin[1]+(random()-0.5)/500],
               [origin[0]+(random()-0.5)/500, origin[1]+(random()-0.5)/500],
               [origin[0]+(random()-0.5)/500, origin[1]+(random()-0.5)/500]]

    return latlngs, origin


######################################################################
# The thread classes below spoof fake random nav data for fake platforms
# It provides an example of the data structures that are required to feed real updates to the map
# TODO: create a similar class that fills out the data structure with real data
# It may be cleaner to keep the class another file that is imported into this one
class FakeAUVThread (threading.Thread):
    def __init__(self, platform, delay):
        threading.Thread.__init__(self)
        self.platform = platform
        self.delay = delay
        self.daemon = True  # run in daemon mode to allow for ctrl+C exit
        self.radius = randint(30, 70)

    def run (self):
        o = [-33.84119 , 151.25612]       # fake origin to bounce around
        i = 0

        while(1) :
            msgid=time.time()  # timestamp / msgid
            lat, lon, hdg = FakeCoordOnCircle(i, self.radius, o)
            i += 1

            platformdata[self.platform] = {
                'msgid': msgid,                                 # REQUIRED (number)
                'pose': {
                    'lat': round(lat, 10),  # round(o[0]+(random()-0.5)/600, 12),  # REQUIRED (decimal degrees)
                    'lon': round(lon, 10),  # round(o[1]+(random()-0.5)/600, 12),  # REQUIRED (decimal degrees)
                    'heading': round(hdg, 1),  # randint(0, 360),                 # REQUIRED (degrees)
                    'alt': round(random()*10, 2),               # OPTIONAL (m)
                    'depth': round(random()*1000, 2),           # OPTIONAL (m)
                    'roll': randint(-20, 20),                   # OPTIONAL / REQUIRED for dashboard (degrees)
                    'pitch': randint(-45, 45),                  # OPTIONAL / REQUIRED for dashboard (degrees)
                    'uncertainty': randint(1, 20)
                },
                'stat': {
                    # You can add whatever custom status messages you like here
                    # NB: 'bat' has a special display widget
                    # Any others, just get displayed normally
                    'bat': randint(0, 100),                     # REQUIRED for dashboard (int 0-100)
                    'waypt': randint(1, 10),
                    'state': 'OK',
                    'lag': 10
                },
                'alert': {
                    # You can add whatever custom alerts you like here.
                    # Must return 1 for OK, anything else shows as error.
                    'leak': randint(0, 1),
                    'power': randint(0, 1),
                    'state': randint(0, 1),
                    'coms': randint(0, 1),
                    'status': randint(0, 1),
                    'lag':0
                }
            }
            time.sleep(self.delay)


class FakeShipThread (threading.Thread):
    def __init__(self, platform, delay):
        threading.Thread.__init__(self)
        self.platform = platform
        self.delay = delay
        self.daemon = True  # run in daemon mode to allow for ctrl+C exit
        self.radius = randint(30, 70)

    def run (self):
        o = [-33.84119 , 151.25612]       # fake origin to bounce around
        i = 0

        while(1) :
            msgid=time.time()   # timestamp / msgid
            lat, lon, hdg = FakeCoordOnCircle(i, self.radius, o)
            i += 1
            platformdata[self.platform] = {
                'msgid': msgid,                                 # REQUIRED (number)
                'pose': {
                    'lat': round(lat, 10),  # round(o[0]+(random()-0.5)/600, 12),  # REQUIRED (decimal degrees)
                    'lon': round(lon, 10),  # round(o[1]+(random()-0.5)/600, 12),  # REQUIRED (decimal degrees)
                    'heading': round(hdg, 1)  # randint(0, 360)                  # REQUIRED (degrees)
                }
            }
            time.sleep(self.delay)



def FakeCoordOnCircle(i, radius, o):
    N = 101
    radius += (random()*2-1)
    angle = math.pi*2*(i % N)/N
    dx = radius*math.cos(angle)
    dy = radius*math.sin(angle)
    lat = o[0] + (180/math.pi)*(dy/6378137)
    lon = o[1] + (180/math.pi)*(dx/6378137)/math.cos(o[0]*math.pi/180)
    hdg = (- math.atan2((lat-o[0]), (lon-o[1]))*180/math.pi + 360) % 360

    return lat, lon, hdg