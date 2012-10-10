#!/usr/bin/env python
import lcm

import os
#import libardrone
import math
import select
import sys
import time
import pygame

# useful paths
SCRIPT_PATH             = os.path.dirname(__file__)
SCRIPT_NAME             = os.path.basename(sys.argv[0])
PATH_TO_PERLS           = os.path.join(SCRIPT_PATH, '..', '..')
PATH_TO_PYTHON_LCMTYPES = os.path.join(PATH_TO_PERLS, 'build','lib','python2.6','dist-packages','perls', 'lcmtypes')

#sys.path.append (PATH_TO_PYTHON_LCMTYPES)
#import perllcm
#import bot_core
from perls import *

# useful constants
MMTOM = 1e-3
DTOR  = math.pi/180
IMG_WIDTH_FORWARD,IMG_HEIGHT_FORWARD = 320, 240
IMG_WIDTH_BOTTOM,IMG_HEIGHT_BOTTOM = 176, 144
BOTTOM_CAM_SIZE = 76032

# class ARState
class ARData:
    '''ARDrone data class'''
    def __init__ (self):
        self.state = dict(roll=0,pitch=0.0,yaw=0.0,vx=0.0,vy=0.0,vz=0.0,altitude=0.0)
        self.battery  = 0.0
        self.in_air = False
        self.coms = False
    
    def publish (self, lc):
        msg = lcmtypes.perllcm.ardrone_state_t ()
        msg.utime    = int(time.time()*1e6)
        msg.roll     = self.state.get('roll',0)
        msg.pitch    = self.state.get('pitch',0)
        msg.yaw      = self.state.get('yaw',0)
        msg.vx       = self.state.get('vx',0)
        msg.vy       = self.state.get('vy',0)
        msg.vz       = self.state.get('vz',0)
        msg.altitude = self.state.get('altitude',0)
        msg.battery  = self.battery
        msg.flying   = self.in_air
        msg.coms     = self.coms
        lc.publish ('ARDRONE_STATE',msg.encode())

# class ARDrone
class ARDriver:
    '''ARDrone driver class'''
    def __init__ (self):
        self.pygame   = False
        self.screen   = ""
        self.prev_img = ""
        self.speed    = 0.2
        self.lc       = lcm.LCM ()
        self.drone    = libardrone.ARDrone ()
        self.running  = True
        self.data     = ARData ()
        self.cam      = True
        self.lc.subscribe ('ARDRONE_CMD', self.cmd_handler)
        self.lc.subscribe ('ARDRONE_DRIVE', self.drive_handler)
        self.takeoff_cmd = False
        
    def add_subscription (self, channel, handler):
        self.lc.subscribe (channel, handler)

    def query_state (self):
        try:
            st, bat, fly, coms = ARData(), [], [], []
            # vehicle data
            st.state['roll']     = self.drone.navdata.get(0,dict()).get('phi',0)*DTOR
            st.state['pitch']    = self.drone.navdata.get(0,dict()).get('theta',0)*DTOR
            st.state['yaw']      = self.drone.navdata.get(0,dict()).get('psi',0)*DTOR
            st.state['vx']       = self.drone.navdata.get(0,dict()).get('vx', 0)*MMTOM
            st.state['vy']       = self.drone.navdata.get(0,dict()).get('vy', 0)*MMTOM
            st.state['vz']       = self.drone.navdata.get(0,dict()).get('vz', 0)*MMTOM
            st.state['altitude'] = self.drone.navdata.get(0,dict()).get('altitude',0)*MMTOM
            bat = self.drone.navdata.get(0,dict()).get('battery',0)
            fly = True if self.drone.navdata.get('drone_state',dict()).get('fly_mask',0) else False
            coms = True if not self.drone.navdata.get('drone_state',dict()).get('com_watchdog_mask',0) else False
            if fly and not coms:
                self.drone.at (at_comwdg)  
        except:
           print 'error querying drone state!'
        else:
            self.data.state    = st.state
            self.data.battery  = bat
            self.data.in_air   = fly
            self.data.coms     = coms
            self.data.publish (self.lc)
 
    def query_camera (self):
        img = lcmtypes.bot_core.image_t ()
        img.utime       = self.drone.utime
        img.pixelformat = img.PIXEL_FORMAT_RGB
        img.size        = len(self.drone.image)
        img.data        = self.drone.image
        sync = lcmtypes.bot_core.image_sync_t ()
        sync.utime = self.drone.utime

        if img.size == BOTTOM_CAM_SIZE:
            img.width       = IMG_WIDTH_BOTTOM
            img.height      = IMG_HEIGHT_BOTTOM
            img.row_stride  = 3*IMG_WIDTH_BOTTOM
        else:
            img.width       = IMG_WIDTH_FORWARD
            img.height      = IMG_HEIGHT_FORWARD
            img.row_stride  = 3*IMG_WIDTH_FORWARD

        if self.pygame:
            surface = pygame.image.fromstring(self.drone.image, (img.width,img.height), 'RGB')
            self.screen.blit(surface, (0,0))
            pygame.display.flip ()
        if img.data != self.prev_img:
            self.prev_img = img.data
            self.lc.publish ('ARDRONE_CAM',img.encode())
            self.lc.publish ('ARDRONE_SYNC', sync.encode())
        
    def change_cam (self):
        if self.cam:
            self.drone.at (libardrone.at_config, "video:video_channel", "1")
        else:
            self.drone.at (libardrone.at_config, "video:video_channel", "0")
        self.cam = not self.cam


    def halt (self):
        self.drone.at (libardrone.at_config, "video:video_channel", "0")
        if (self.data.in_air):
            self.drone.land ()
        self.drone.halt ()
        del self.drone
        self.running = False
    
    def update_cmd (self):
        if self.takeoff_cmd and not self.data.in_air:
            self.drone.takeoff ()
        if not self.takeoff_cmd and self.data.in_air:
            self.drone.land ()        
        elif self.data.in_air: 
            vx = 0.0 if abs(self.vx_cmd) < 0.001 else self.vx_cmd*self.speed
            vy = 0.0 if abs(self.vy_cmd) < 0.001 else self.vy_cmd*self.speed
            vz = 0.0 if abs(self.vz_cmd) < 0.001 else self.vz_cmd
            vr = 0.0 if abs(self.vr_cmd) < 0.001 else self.vr_cmd 
            if vx or vy or vz or vr:         
                self.drone.move (vy, vx, vz, vr)
		#print '%f,\t%f,\t%f,\t%f' % (vy, vx, vz, vr)
            else:
                self.drone.hover ()

        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.vz_cmd = 0.0
        self.vr_cmd = 0.0
              

    def cmd_handler (self, channel, data):
        cmd = lcmtypes.perllcm.ardrone_cmd_t.decode (data)
        if cmd.takeoff:
            self.takeoff_cmd = not self.takeoff_cmd
        if cmd.camera:
            self.change_cam ()
        if cmd.emergency:
            self.drone.reset ()
            self.takeoff_cmd = False 
            self.data.in_air = False       
        if cmd.change_speed > 0:
            self.speed = self.speed + 0.1
            if self.speed > 1.0:
                self.speed = 1.0
        elif cmd.change_speed < 0 and self.speed > 0.2:
            self.speed = self.speed - 0.1

    def drive_handler (self, channel, data):
        drive = lcmtypes.perllcm.ardrone_drive_t.decode (data)
        self.vx_cmd = drive.vx
        self.vy_cmd = drive.vy
        self.vz_cmd = drive.vz
        self.vr_cmd = drive.vr


def main():
    driver = ARDriver () 
    # start the party
    count = 0
    if len(sys.argv) >= 2:
        driver.pygame = True if sys.argv[1] == "-p" else False
    if driver.pygame:
        pygame.init ()
        driver.screen = pygame.display.set_mode((320,240))

    while driver.running:
        try:
            # listen to drone command channel with timeout
            now = time.time()
            while time.time() - now  < 0.025:
                inready,outread,exceptready=select.select([driver.lc.fileno()],[],[],0.025)
                if inready:
                    for n in inready:
                        driver.lc.handle ()
            if count % 3 == 0:        
                driver.update_cmd ()
                driver.query_state ()

            driver.query_camera ()
            count = count+1
        except KeyboardInterrupt:
            print 'caught SIGINT!'
            driver.halt ()
        except:
            pass


    # make sure drone has landed and is safe
    print 'halting drone...',
    print 'done'

if __name__ == '__main__':
    sys.exit (main())

