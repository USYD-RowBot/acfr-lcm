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
import json
import time
import requests
import threading

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

configfile = "config/{}-{}.ini".format(socket.gethostname(),module)
allowadmin = "false"
welcomenote = ""

@app.route('/')
def home():
    return render_template('index.html', configfile=configfile, allowadmin=allowadmin, welcomenote=welcomenote)
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


@app.route('/setall_platformdata', methods=['POST'])
def setall_platformdata():
    data = request.form.get('platformdata')
    pd.set_platformdata(data=data)
    return jsonify({"result": "ok"})

@app.route('/set_platformdata', methods=['POST'])
def set_platformdata():
    platform = request.args.get('platform')
    data = request.form.get('platformdata')
    pd.set_platformdata(platform=platform, data=data)
    return jsonify({"result": "ok"})


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


@app.route('/send_to_platform', methods=["POST"])
def send_to_platform():
    # platform as a GET argument
    #thisplatform = request.args.get('platform')
    args = dict(request.form)
    platform, response = pd.send_to_platform(args)

    return jsonify({"result": response, "platform": platform})


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


def get_platform_cmd_form(platform):
    return '<div><b><i class="fa fa-gears"></i> Config File Editor</b></div>Command options for: '+platform



class sendRemoteDataThread (threading.Thread):
    def __init__(self, delay, targets, destserver):
        threading.Thread.__init__(self)
        self.delay = delay
        self.targets = targets
        self.destserver = destserver
        self.daemon = True  # run in daemon mode to allow for ctrl+C exit

    def run(self):

        while(1) :
            sendplatforms = {}
            for key in self.targets:
                try:
                    print 'Getting data for {}'.format(key)
                    r = requests.get(url=self.targets[key])
                    if r.status_code == 200:
                        sendplatforms[key] = json.loads(r.text)
                    print "Received data for: {}".format(self.targets[key])
                except:
                    print "ERROR!!!   Cannot get data for: {}".format(self.targets[key])

            try:
                print "Sending data to {}".format(self.destserver)
                payload = {'platformdata': json.dumps(sendplatforms)}
                filename = "logs/{}-{}.json".format(module, int(time.time()))
                f = open(filename, 'w')
                f.write(payload['platformdata'])
                f.close()
                r = requests.post(self.destserver, data=payload)
            except:
                print "ERROR!!!   Unable to send data to {}".format(self.destserver)

            time.sleep(self.delay)


def run_server_cfg(configfile):
    global allowadmin, welcomenote
    cfg = ConfigParser.ConfigParser()
    cfg.read(configfile)
    if (cfg.has_option('server', 'remotepush')):
        remotesec = cfg.get('server', 'remotepush')
        url = cfg.get(remotesec, 'url')

        targets = {}
        for k in cfg.get(remotesec, 'targets').split(','):
            targets[k] = cfg.get(k, 'url')
            if targets[k].find('http://') <= 0:
                targets[k] = "http://localhost:{}/{}".format(port, targets[k])

        print targets
        upddelay = float(cfg.get(remotesec, 'upddelay'))

        sendRemoteDataThread(upddelay, targets, url).start()

    welcomenote = cfg.get('server','welcomenote') if cfg.has_option('server','welcomenote') else ""

    allowadmin = cfg.get('server', 'allowadmin') if (cfg.has_option('server', 'allowadmin')) else "false"

    return





if __name__ == '__main__':
    # Start threads that do update vehicle data
    print "Starting data threads..."
    pd.init_platformdata_threads()
    run_server_cfg(configfile)

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










