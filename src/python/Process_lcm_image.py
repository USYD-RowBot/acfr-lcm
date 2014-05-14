import sys
import lcm

import numpy as np

sys.path.append('/home/navid/proj/acfr/git/acfr_lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
from perls import lcmtypes
from bot_core.image_t import image_t
from acfrlcm.classified_t import classified_t


class Process_lcm_images (object):
    def __init__ (self):
        self.lc = lcm.LCM ()
        self.sub = self.lc.subscribe ("PROSILICA_RM16", self.img_process_callback )
        

    def img_process_callback (self, channel, data):
        '''Callback function to read the next image, do something about it and 
        publish the result back into LCM
        '''
        # READ image from LCM
        msg = lcmtypes.bot_core.image_t.decode (data)
        print ("received image packet, recovering current image data...")
        print "image time=%d, w/h=%d/%d, size=%d" % (msg.utime, msg.width, msg.height, msg.size)

        # Do something

        # How to publish <my class type, here classified> back to LCM
        c = classified_t()
        c.utime = msg.utime
        c.x = msg.width
        c.y = msg.height
        #c.name = "test"
        self.lc.publish ("CLASSIFIED", c.encode ())
        #print ""


    def do_work (self):
        try:
            while True:
                self.lc.handle ()
        except KeyboardInterrupt:
            pass

        self.lc.unsubscribe (self.sub)

if __name__ == "__main__":
    proc = Process_lcm_images ()
    proc.do_work ()

    sys.exit (0)
