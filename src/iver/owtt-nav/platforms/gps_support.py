import sys
import math
import lcm

from perls import BotParam
from perls import BotCore
from perls import lcmtypes

RTOD = 180/math.pi

TDIFF_TOL = 3*1e6 # gps meas must be within TDIFF_TOL usecs

class Gps_navigation_aid (object):
    def __init__ (self, param_args):
        param = BotParam.BotParam (param_args)
        orglatlon = param.get_double_array ("site.orglatlon")
        
        self.llxy = BotCore.BotGPSLinearize (orglatlon[0],orglatlon[1])
        self.xy_utime = 0
        self.xy = []

        self.lc = lcm.LCM ()

        acomms_chan = param.get_str ("sensors.modem.gsd.channel")
        self.rq_sub = self.lc.subscribe (acomms_chan + "_NAV_REQUEST", self.pose_request_cb)
        self.reply_chan = acomms_chan + "_NAV_POSE_REPLY"

        gpsd_chan = param.get_str ("sensors.gpsd3-client.gsd.channel")
        self.gps_sub = self.lc.subscribe (gpsd_chan, self.gpsd_cb)

        self.depth = param.get_int ("owtt-nav.topside.sensors.static_depth")

        self.done = False


    def pose_request_cb (self, channel, data):
        msg = lcmtypes.senlcm.acomms_request_t.decode (data)
        if (msg.type != lcmtypes.senlcm.acomms_request_t.POSE): 
            return
        print ("received pose request --- sending gps position")

        if (msg.utime > self.xy_utime + TDIFF_TOL): 
            print ("gps position stale! is gps running?")
            return

        if not len (self.xy):
            print ("have not received gps position")
            return

        pose = lcmtypes.senlcm.acomms_pose_t ()
        pose.utime = msg.utime
        pose.depth = self.depth
        pose.xy[0] = self.xy[0]
        pose.xy[1] = self.xy[1]
        # should get cov from param or hdop
        pose.xy_cov[0] = 9
        pose.xy_cov[1] = 0
        pose.xy_cov[2] = 0
        pose.xy_cov[3] = 9
        self.lc.publish (self.reply_chan, pose.encode ())

    def gpsd_cb (self, channel, data):
        msg = lcmtypes.senlcm.gpsd3_t.decode (data)

        lat,lon = msg.fix.latitude*RTOD, msg.fix.longitude*RTOD
        y,x = self.llxy.to_xy (lat, lon)
        self.xy = [x,y]
        self.xy_utime = msg.utime

    def run (self):
        try:
            while True:
                self.lc.handle ()
        except KeyboardInterrupt:
            pass

        self.lc.unsubscribe (self.rq_sub)
        self.lc.unsubscribe (self.gps_sub)


if __name__ == "__main__":
    cna = Gps_navigation_aid (sys.argv)

    print ("waiting to send navigation data")
    cna.run ()

    sys.exit (0)








