#!/usr/bin/env python

import sys
import os
import time

import lcm

import perls

BotParam = perls.BotParam.BotParam

senlcm_prosilica_attribute_t = perls.lcmtypes.senlcm.prosilica_attribute_t
senlcm_prosilica_t = perls.lcmtypes.senlcm.prosilica_t
hauv_bs_rnv_2_t = perls.lcmtypes.hauv.bs_rnv_2_t

def timestamp_now ():
    return time.time()*1e6

class ProsilicaController ():

    def __init__(self, lcm, param, periscopeDepth):
        self.depth = 0

        self.lcm = lcm
        self.param = param
        self.periscopeDepth = periscopeDepth

        self.uwChannel = param.get_str (param.get_str ('rtvan.cameraUw') + '.channel')
        self.periChannel = param.get_str (param.get_str ('rtvan.cameraPeri') + '.channel')

        print 'Using %s for underwater prosilica channel' % self.uwChannel
        print '      %s for periscope prosilica channel' % self.periChannel

    def _atPeriscopeDepth(self):
        return (self.depth < self.periscopeDepth)

    def hauv_bs_rnv_2_t_callback (self, channel, data):
        msg = hauv_bs_rnv_2_t.decode (data)
        self.depth = msg.depth
        # print str(msg.depth) + " " + str(self._atPeriscopeDepth())
        self.sendProsilicaAttr ()

    def sendProsilicaAttr (self):
        
        if self._atPeriscopeDepth ():
            self.sendStart (self.periChannel)
            self.sendStop (self.uwChannel)
        else:
            self.sendStart (self.uwChannel)
            self.sendStop (self.periChannel)

    def sendStart (self, channel):
        self.sendCmd ('AcquisitionStart', channel + '.ATTRIBUTES')

    def sendStop (self, channel):
        self.sendCmd ('AcquisitionStop', channel + '.ATTRIBUTES')

    def sendCmd (self, label, channel):
        attrib = senlcm_prosilica_attribute_t ()
        attrib.label = label
        attrib.value = ''

        msg = senlcm_prosilica_t ()
        msg.PvAttributes = [attrib]
        msg.n_attributes = 1
        msg.utime = timestamp_now ()

        self.lcm.publish (channel, msg.encode())

if __name__ == '__main__':

    if len (sys.argv) != 2:
        print 'usage: %s DEPTH' % os.path.basename (sys.argv[0])
        print '  where DEPTH (in meters) is the depth at which an ascending HAUV goes from using the underwater camera to using the periscope camera'
        sys.exit (1)

    lcm = lcm.LCM ()
    param = BotParam ()
    pc = ProsilicaController(lcm, param, float(sys.argv[1]))


    lcm.subscribe ('HAUV_BS_RNV_2', pc.hauv_bs_rnv_2_t_callback)

    print 'Waiting for depth...'
    try:
        while True:
            lcm.handle ()
    except KeyboardInterrupt:
        pass
