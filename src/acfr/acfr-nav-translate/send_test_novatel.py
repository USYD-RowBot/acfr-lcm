import lcm
import time
import sys
import readchar


sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
from acfrlcm import auv_acfr_nav_t
from senlcm import novatel_t



def Main():

    channel = 'WAMV.NOVATEL'
    if len(sys.argv) > 1:
        channel = sys.argv[1] # e.g. 'STARBOARD_MOTOR_CONTROL'

    print('ACFR USYD Nav Tranlate Tester')
    print('LCM channel: {}'.format(channel))
    print('Usage: python send_test_novatel.py <>')
    print(' e.g.  python send_test_novatel.py ____\n')


    # Setup LCM
    lc = lcm.LCM()

    msg = novatel_t()

    msg.utime = long(time.time())*1000000
    msg.latitude = -0.591480163 # -33.889317 deg
    msg.longitude = 2.638794939 # 151.191813 deg
    msg.roll = 0.0 # 0.087266463 # 5.00 deg
    msg.pitch = 0.0 # 0.174532925 # 10.00 deg
    msg.heading = 0.785398163 # 0.261799388 # 15.00 deg
    msg.height = 0.0 # m
    msg.north_velocity = 2.0 # m/s
    msg.east_velocity = 2.0 # m/s
    msg.up_velocity = 0.0 # m/s
    msg.latitude_sd = 0.0
    msg.longitude_sd = 0.0
    #msg.string status = 'test'

    lc.publish(channel, msg.encode())

if __name__ == '__main__':
    Main()

