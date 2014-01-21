import sys
import lcm

import numpy as np

from perls import lcmtypes

class Publish_acomms_pose (object):
    def __init__ (self):
        self.lc = lcm.LCM ()
        self.sub = self.lc.subscribe ("TOPSIDE_ACOMMS_NAV_OSP", self.nav_osp_callback)
        #self.sub = self.lc.subscribe ("IVER28_ACOMMS_NAV_OSP", self.nav_osp_callback)
        #self.sub = self.lc.subscribe ("IVER31_ACOMMS_NAV_OSP_REPLY", self.nav_osp_callback)

    def nav_osp_callback (self, channel, data):
        msg = lcmtypes.senlcm.acomms_two_osp_t.decode (data)
        print ("received osp packet, recovering current pose marginal...")
        print ("current: new = %d, org = %d, last: new = %d, org = %d" % (msg.current.new_tol_no, msg.current.org_tol_no, msg.last.new_tol_no, msg.last.org_tol_no))

        Lambda = np.array ([msg.current.Lambda[0:4],
                            msg.current.Lambda[4:8],
                            msg.current.Lambda[8:12],
                            msg.current.Lambda[12:16] ])
        eta = np.array (msg.current.eta)

        #print  "COVARIANCE"
        Sigma = np.linalg.inv (Lambda)
        mu = np.dot (Sigma, eta)
        #print Sigma
        #print mu

        pose = lcmtypes.perllcm.position_t ()
        pose.utime = msg.utime
        pose.xyzrph[0] = mu[0]
        pose.xyzrph[1] = mu[1]
        pose.xyzrph[2] = msg.current.depth
        pose.xyzrph_cov[0] = Sigma[0,0]
        pose.xyzrph_cov[1] = Sigma[0,1]
        pose.xyzrph_cov[6] = Sigma[1,0]
        pose.xyzrph_cov[7] = Sigma[1,1]
        self.lc.publish ("TOPSIDE_SERVER_OSP_POSE", pose.encode ())
        print ""

    def do_work (self):
        try:
            while True:
                self.lc.handle ()
        except KeyboardInterrupt:
            pass

        self.lc.unsubscribe (self.sub)

if __name__ == "__main__":
    pub = Publish_acomms_pose ()
    pub.do_work ()

    sys.exit (0)
        
