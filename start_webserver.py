#!/usr/bin/python

######################################################################
# Simple webserver that runs mission viewer.
#
# REQUIREMENTS: to run this code make sure you have the following
# dependencies:
#   sudo pip install Flask
#   sudo pip install flask-jsonpify     # to support cross domain json calls
#   sudo pip install Pillow             # (not core) python image lib for geotiff conversion
#   sudo pip install gdal               # (not core) for handling geotiffs
#   sudo pip install pyproj             # (not core) for handling projections
#
# HOW TO RUN:
#   python start_webserver.py test
#               OR
#   python start_webserver.py acfr
#
# HOW TO TEST:
# If you have the UI code, open a web browser and go to:
#   localhost:8080/
# If you do not have the UI code, you can still test that the  platform
# data threads are running properly, open a web browser and go to:
#   localhost:8080/get_platformdata?platform=0
# where "0" is the ID of the platform you would like to get data for.
# You should see a JSON message containing the data structure that
# should update on refresh.
#
# NOTE:
# If you would like to add your platform to the mission viewer, you
# have come to the right place, but you're in the wrong file.
# For anything basic, you shouldn't need to edit this file. The file
# you are looking for is:
#   platformdata_thread.py
#
# That file does all the heavy lifting for platform pose updates.There
# should be enough details in there to help. If it is not obvious please
# contact me.
#
# Author:
#   Ariell Friedman
#   ariell.friedman@gmail.com
#   25 AUG 2014
#
######################################################################

from flask import Flask, Response, request, render_template, url_for, send_from_directory #, jsonify
from flask.ext.jsonpify import jsonify   # allow cross-domain jsonp messages (for use across the network)
import socket  # to get local hostname and ip address easily
import ConfigParser
from osgeo import gdal      # for geotiff reading
from PIL import Image       # for geotif conversion
import os
import sys


app = Flask(__name__)

# automatically work out IP address
ipaddress = "%s.local" % socket.gethostname()
try:
    ipaddress = socket.gethostbyname(ipaddress)
except:
    pass
thisserver = "http://%s:8080" % ipaddress

# import all the worker threads and functions to deal with platform data updates
mode = sys.argv[1] if len(sys.argv) > 1 else "test"
exec "from platformdata.{} import *".format(mode)


@app.route('/')
def home():
    return render_template('index.html', server=thisserver)
    #return render_template('index.html', server="http://localhost:8080")  # server is this machine

@app.route('/get_mission')
def get_mission():
    filepath = request.args.get('filepath')  # mission file path
    olat = request.args.get('olat')  #
    olon = request.args.get('olon')  #

    latlngs, origin = parse_mission(filepath, [olat, olon])

    return jsonify({"latlngs": latlngs, "origin": origin})


@app.route('/get_config')
def get_config():
    cfg = request.args.get('cfg')
    sec = request.args.get('sec')
    config = ConfigParser.ConfigParser()
    config.read(cfg)
    if sec is None:
        return jsonify(config.sections())
    else:
        dict = {}
        for opt in config.options(sec):
            dict[opt] = config.get(sec, opt)
        return jsonify(dict)


@app.route('/get_geotiff')
def get_geotiff():
    # This function needs to get a url for a geotiff, compute the bounds and return the rquired variables as json
    url = request.args.get('url')
    chartdirurl = 'uploads/geotiffs/'

    ds = gdal.Open(url)
    gt = ds.GetGeoTransform()
    bounds = [
        [gt[3], gt[0]],
        [gt[3] + ds.RasterXSize*gt[4] + ds.RasterYSize*gt[5], gt[0] + ds.RasterXSize*gt[1] + ds.RasterYSize*gt[2]]
    ]

    # Leaflet cannot display geotiffs in the browser, so we need to convert them to pngs
    imgurl = chartdirurl+os.path.basename(url)+'.png'
    if not os.path.isfile(imgurl):
        Image.open(url).save(imgurl)

    return jsonify({'bounds': bounds, 'imgurl': imgurl})


@app.route('/get_platformdata')
def platformdata():
    # platform as a GET argument
    thisplatform = request.args.get('platform')
    # get_platformdata is a function contained in included python script
    return jsonify(get_platformdata(thisplatform))


@app.route('/set_waypoint')
def set_waypoint():
    # platform as a GET argument
    thisplatform = request.args.get('platform')
    lat = request.args.get('lat')
    lon = request.args.get('lon')

    platform, response = send_waypoint(thisplatform, lat, lon)

    return jsonify({"result": response, "platform": platform})


# Custom static data
@app.route('/uploads/<path:filename>')
def serve_uploads(filename):
    return send_from_directory('uploads', filename)


if __name__ == '__main__':

    # Start threads that do update vehicle data
    print "Starting data threads..."
    init_platformdata_threads()

    print "Starting webserver..."
    print "To connect to this server from another machine on the network, open a browser and go to: \n\n    {}\n".format(thisserver)
    #print app.config
    app.run(
        #debug = True,
        host = "0.0.0.0",
        port = 8080
    )

    terminate_platformdata_threads()

    exitFlag = 1

