from random import random, randint
import time
import threading

import sys
# import math
# import ConfigParser
# import select
# import string
import os
import pyproj
import glob



import missionMP


def parse_mission (filepath, cfgorigin=[0, 0]):
    # TODO: parse mission file and origin and return in LAT/LON
    # filepath will contain the filepath to the mission file

    # from the filename we need to work out which mission parser to use
    fileName, fileExtension = os.path.splitext(filepath)
    mission = missionMP.Mission()

    mission.load(filepath)
    #origin = [mission.getOriginLat(), mission.getOriginLon()]
    #if origin[0] == 0 or origin[1] == 0:
    #    origin = cfgorigin
    origin = cfgorigin

    waypoints = mission.dumpSimple()

    # convert the waypoints to latitude and longitude
    projStr = '+proj=tmerc +lon_0={} +lat_0={} +units=m'.format(origin[1], origin[0])
    p = pyproj.Proj(projStr)

    latlngs = []
    for wpt in waypoints:
        ll = p(wpt.y, wpt.x, inverse=True)
        latlng = [ll[1], ll[0]]
        latlngs.append(latlng)

    return latlngs, origin

def get_mission_start (path) :
    lat = os.popen("cat {}/d*/*localiser.cfg | grep ^LATITUDE  | tr -s ' ' | cut -d' ' -f2".format(path)).read().strip()
    lon = os.popen("cat {}/d*/*localiser.cfg | grep ^LONGITUDE  | tr -s ' ' | cut -d' ' -f2".format(path)).read().strip()
    mpfile = os.popen("ls {}/d*/*.mp".format(path)).read().strip()
    if mpfile:
        mpcoords =  parse_mission(mpfile, [lat, lon])
        return mpcoords[0][0],mpcoords[1]
    return False, False

missions = glob.glob("{}/r*".format(sys.argv[1]))
f = open('mission_origins.txt','w')
for m in missions:
    try:
        start,origin = get_mission_start(m)
        f.write('{}\nstart: {}, {}\norigin: {}, {}\n'.format(m,start[0],start[1],origin[0],origin[1]))
    except:
        print "ERROR!!!!!!!!!!!!!!!!!!"
f.close()  # you can omit in most cases as the destructor will call if