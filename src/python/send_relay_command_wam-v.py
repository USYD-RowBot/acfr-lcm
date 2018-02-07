import lcm
import time
import sys
import readchar

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
from acfrlcm import relay_command_t


def Main():

    channel = 'WAMV.RELAY_CONTROL'

    print('ACFR USYD dS2824 Relay Command Utility - LCM channel: {}'.format(channel))
    if len(sys.argv) < 4:
        print('Usage: python send_relay_command_wam-v.py <R/O> <relay number> <0/1>')
        print(' e.g.  python send_relay_command_wam-v.py R 8 1     (to turn relay 8 on)')
        print('       python send_relay_command_wam-v.py O 5 0     (to turn i/o 5 off)')
        print('       python send_relay_command_wam-v.py D 12 1000 (to turn relay 12 on for 1000 ms))')
        print('Refer to config file for relay number to device listing\n')
        sys.exit()

    # Setup LCM
    lc = lcm.LCM()

    msg = relay_command_t()

    if str(sys.argv[1]) == 'R':
         msg.relay_number = int(sys.argv[2])
         msg.relay_request = int(sys.argv[3])
         msg.relay_off_delay = 0
         msg.io_number = 0
         msg.io_request = 0
    elif str(sys.argv[1]) == 'O':
         msg.io_number = int(sys.argv[2])
         msg.io_request = int(sys.argv[3])
         msg.relay_off_delay = 0
         msg.relay_number = 0
         msg.relay_request = 0
    elif str(sys.argv[1]) == 'D':
         msg.relay_number = int(sys.argv[2])
         msg.relay_request = 1
         msg.relay_off_delay = int(sys.argv[3])
         msg.io_number = 0
         msg.io_request = 0

    msg.utime = long(time.time())*1000000
    lc.publish(channel, msg.encode())

if __name__ == '__main__':
    Main()


