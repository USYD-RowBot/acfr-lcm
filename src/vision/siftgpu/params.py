#!/usr/bin/env python

import time
from optparse import OptionParser

import perls
import lcm

SIFTGPU_PARAMS = "SIFTGPU_PARAMS"

SiftParams = perls.lcmtypes.perllcm.van_siftgpu_params_t

if __name__ == '__main__':
    
    msg = SiftParams()

    #Sending these parameters seems to have an effect for CUDA-enabled systems.  
    # NOTE: If siftgpu was compiled without CUDA support, these values do nothing
    msg.dogThreshold = 0.02
    msg.edgeThreshold = 10.0
    msg.dogLevels = 3
    msg.filterWidthFactor = 4.0

    #...these parameters do not seem to have an effect even on CUDA-enabled systems.
    msg.orientWindowFactor = 2.0
    msg.gridSizeFactor = 3.0

    lcm = lcm.LCM()
    lcm.publish(SIFTGPU_PARAMS, msg.encode())
