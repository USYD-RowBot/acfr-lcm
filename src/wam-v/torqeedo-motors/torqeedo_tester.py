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
    print('                  5            ')
    print('                 idle          ')
    print('                  2  -         ')

    # Start with both neg -> idle
    cmd_speed = 0

    # Setup LCM
    lc = lcm.LCM()

    msg = asv_torqeedo_motor_command_t()

    while True:

        time.sleep(0.1)
        choice = int((readchar.readkey()))
        if choice == 8:
            # increase speed ^
            cmd_speed = (cmd_speed + 10) #%(650)
        elif choice == 2:
            cmd_speed = (cmd_speed - 10)
        elif choice == 5:
            # reverse <
            cmd_speed = 0

        msg.command_speed = cmd_speed
        msg.utime = long(time.time())*1000000
        lc.publish(channel, msg.encode())
        

if __name__ == '__main__':
    Main()


