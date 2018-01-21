import lcm
import time
import sys
import readchar

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
from acfrlcm import relay_command_t


def Main():

    channel = 'RELAY_CONTROL'

    print('ACFR USYD dS2824 Relay TESTER started for driver LCM channel: {}'.format(channel))
    if len(sys.argv) < 3:
        print('Usage: python relay_tester.py <relay number> <0/1>')
        print(' e.g.  python relay_tester.py 8 1    (to turn relay 8 on)\n')
        sys.exit()

    # Setup LCM
    lc = lcm.LCM()

    msg = relay_command_t()

    msg.relay_number = int(sys.argv[1])
    msg.relay_request = int(sys.argv[2])
    msg.relay_off_delay = 0
    msg.io_number = 0
    msg.io_request = 0
    msg.utime = long(time.time())*1000000
    lc.publish(channel, msg.encode())

if __name__ == '__main__':
    Main()


