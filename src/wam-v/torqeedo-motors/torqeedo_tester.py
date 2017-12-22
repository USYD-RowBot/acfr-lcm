import lcm
import time
import sys
import readchar


sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
from acfrlcm import asv_torqeedo_motor_command_t


def Main():

    global f_speed
    global r_speed

    channel = 'PORT_MOTOR_CONTROL'
    if len(sys.argv) > 1:
        channel = sys.argv[1] # e.g. 'STARBOARD_MOTOR_CONTROL'

    print('ACFR USYD Torqeedo Motor TESTER started for driver LCM channel: {}'.format(channel))
    print('Usage: python torqeedo_tester.py <motor_lcm_channel>')
    print(' e.g.  python torqeedo_tester.py PORT_MOTOR_CONTROL\n')
    print('Motor Control:    8  + \n      ')  
    print('          rev 4   5   6 fwd    ')
    print('                 idle          ')
    print('                  2  -         ')

    # Start with both neg -> idle
    f_speed = -1
    r_speed = -1

    # Setup LCM
    lc = lcm.LCM()

    msg = asv_torqeedo_motor_command_t()

    while True:

        time.sleep(0.1)
        choice = int((readchar.readkey()))
        if choice == 8:
            # increase speed ^
            if f_speed >= 0:
                f_speed = (f_speed + 1) #%(650)
            elif r_speed >= 0:
                r_speed = (r_speed +1) #%(590)
        elif choice == 2:
            # decerease speed v
            if f_speed >= 0:
                f_speed = (f_speed - 1)
            elif r_speed >= 0:
                r_speed = (r_speed - 1)
        elif choice == 4:
            # reverse <
            f_speed = -1
            r_speed = 20
        elif choice == 6:
            # forward >
            f_speed = 20
            r_speed = -1
        elif choice == 5:
            # stopped _
            f_speed = -1
            r_speed = -1

        msg.forward_speed = f_speed
        msg.reverse_speed = r_speed
        msg.utime = long(time.time())*1000000
        lc.publish(channel, msg.encode())
        

if __name__ == '__main__':
    Main()


