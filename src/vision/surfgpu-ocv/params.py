#!/usr/bin/env python

import time
from optparse import OptionParser

import perls
import lcm

SURFGPU_PARAMS = "SURFGPU_PARAMS"

SurfParams = perls.lcmtypes.perllcm.van_cvsurf_params_t

if __name__ == '__main__':

    #Parse the options from command line
    parser = OptionParser()
    parser.add_option('-t', 
                      type="float",
                      dest="hessianThreshold", 
                      help="The Hessian threshold",
                      default=500.0)
    parser.add_option('-s', 
                      action="store_false",
                      dest="extended", 
                      help="If provided, use 64-length descriptor instead of 128",
                      default=True)
    parser.add_option('-n', 
                      type="int",
                      dest="nOctaves", 
                      help="Number of octaves",
                      default=4)
    parser.add_option('-l', 
                      type='int',
                      dest="nOctaveLayers", 
                      help="Number of octave layers",
                      default=2)

    (options, args) = parser.parse_args()


    #Populate the lcm message
    msg = SurfParams()
    msg.extended = options.extended
    msg.hessianThreshold = options.hessianThreshold
    msg.nOctaves = options.nOctaves
    msg.nOctaveLayers = options.nOctaveLayers
    msg.utime = int(time.time() * 1000000)

    #Publish the lcm message
    lcm = lcm.LCM()
    lcm.publish(SURFGPU_PARAMS, msg.encode())
