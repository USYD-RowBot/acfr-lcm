#!/usr/bin/python

######################################################################
# Simple webserver that runs mission viewer.
#
# REQUIREMENTS: to run this code make sure you have the following
# dependencies:
#   sudo pip install -r requirements.txt
#
# HOW TO RUN:
#   python start_webserver.py test
#               OR
#   python start_webserver.py acfrauv
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

from flask import Flask, Response, request, render_template, url_for, send_from_directory, redirect #, jsonify
from flask.ext.jsonpify import jsonify   # allow cross-domain jsonp messages (for use across the network)
import socket  # to get local hostname and ip address easily
import ConfigParser
from osgeo import gdal      # for geotiff reading
from PIL import Image       # for geotif conversion
import os
import sys
import shutil
from gevent.wsgi import WSGIServer    # host it properly



app = Flask(__name__)



# import all the worker threads and functions to deal with platform data updates
module = sys.argv[1] if len(sys.argv) > 1 else "test"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 8080
pd = __import__("platformdata.{}".format(module))
pd = getattr(pd, module)

# automatically work out IP address
ipaddress = "%s" % socket.gethostname()
try:
    ipaddress = socket.gethostbyname(ipaddress)
except:
    pass
thisserver = "http://{}:{}".format(ipaddress, port)


@app.route('/')
def home():
    return render_template('index.html', hostname=socket.gethostname())
    #return render_template('index.html', server="http://localhost:8080")  # server is this machine

@app.route('/get_mission')
def get_mission():
    filepath = request.args.get('filepath')  # mission file path
    olat = request.args.get('olat')  #
    olon = request.args.get('olon')  #

    latlngs, origin = pd.parse_mission(filepath, [olat, olon])

    return jsonify({"latlngs": latlngs, "origin": origin})


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
def get_platformdata():
    # platform as a GET argument
    thisplatform = request.args.get('platform')
    # get_platformdata is a function contained in included python script
    return jsonify(pd.get_platformdata(thisplatform))

# Custom static data
@app.route('/uploads/<path:filename>')
def serve_uploads(filename):
    return send_from_directory('uploads', filename)



@app.route('/get_config', methods=['GET', 'POST'])
def get_config():
    cfg = request.args.get('cfg')
    if request.method == 'POST':
        cfgtext = request.form.get('cfgtxt')
        f = open(cfg, 'w')
        f.write(cfgtext)
        f.close()
        return redirect("/")
    else:
        sec = request.args.get('sec')
        if (not os.path.isfile(cfg)):  # copy default config if it doesn't exist
            shutil.copy2('{}/template.ini'.format(os.path.dirname(cfg)), cfg)

        config = ConfigParser.ConfigParser()
        config.read(cfg)
        if sec is None:
            return get_config_form(cfg)
        elif sec == "all":
            return jsonify(config.sections())
        else:
            dict = {}
            for opt in config.options(sec):
                dict[opt] = config.get(sec, opt)
            return jsonify(dict)


def get_config_form (cfg) :
    f = open(cfg, 'r')
    cfgtext = f.read()
    f.close()
    form = '<form id="modalform" method="post" action="get_config?cfg={0}"><div><b><i class="fa fa-gears"></i> Config File Editor</b></div>'\
        +'<input name="cfgfile" id="cfgfile" style="width: 100%; border: none; background-color: #EEE; margin-bottom: 2px;" value="{0}" disabled>'\
        +'<textarea name="cfgtxt" id="cfgtxt" style="width: 100%; border: none; height: 450px; background-color: #EEE; font: 11px \'Courier New\', courier, monospace;">{1}</textarea>'\
        +'<button class="btn btn-primary pull-right" type="submit" title="Update">Update</button>'\
        +'<button class="btn btn-primary" type="button" title="Cancel" data-dismiss="modal">Cancel</button></form>'

    return form.format(cfg, cfgtext)


@app.route('/send_to_platform', methods=["POST"])
def send_to_platform():
    # platform as a GET argument
    #thisplatform = request.args.get('platform')
    args = dict(request.form)
    platform, response = pd.send_to_platform(args)

    return jsonify({"result": response, "platform": platform})

def get_platform_cmd_form (platform):
    return '<div><b><i class="fa fa-gears"></i> Config File Editor</b></div>Command options for: '+platform


if __name__ == '__main__':

    # Start threads that do update vehicle data
    print "Starting data threads..."
    pd.init_platformdata_threads()

    print "Starting webserver..."
    print "To connect to this server from another machine on the network, open a browser and go to: \n\n    {}\n".format(thisserver)
    #print app.config
    # app.run(
    #     #debug = True,
    #     host = "0.0.0.0",
    #     port = 8080
    # )

    http_server = WSGIServer(('', port), app)
    http_server.serve_forever()

